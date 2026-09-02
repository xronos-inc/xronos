// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// These tests drive the shared engine in the same way the SDK does. They
// own a `backend::Engine` and drive the lifecycle through its members,
// while assembly and hot-path lookups go through the frozen `abi::Backend`
// view (`abi()`) exactly like the SDK's version-skewed inline glue. No
// value is checked inside a reaction handler (handlers may run on runtime
// worker threads, where Catch2 assertions are not safe); handlers only
// record, and the checks run after the run returns.

#include <algorithm>
#include <array>
#include <atomic>
#include <barrier>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"
#include "xronos/abi/backend.hh"
#include "xronos/abi/exceptions.hh"
#include "xronos/abi/types.hh"
#include "xronos/backend/backend.hh"
#include "xronos/backend/engine.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/runtime/default/default_runtime.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/value/boxing.hh"

namespace {

using namespace std::chrono_literals;
using xronos::abi::ElementUid;
using xronos::core::BoundaryCrossing;
using xronos::runtime::ExecutionProperties;

// Bridges plain callables to the ABI's owned callback interfaces for tests.
template <class Interface> class FunctionCallback final : public Interface {
public:
  explicit FunctionCallback(std::function<void()> fn)
      : fn_{std::move(fn)} {}
  void invoke() final { fn_(); }

private:
  std::function<void()> fn_;
};

template <class Interface, class Fn> auto make_callback(Fn&& fn) -> Interface* {
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): ownership transfers to the backend.
  return new FunctionCallback<Interface>{std::forward<Fn>(fn)};
}

// Runs the engine on a fresh default runtime, the way the SDK's backend
// composes a run.
void run(xronos::backend::Engine& engine, const ExecutionProperties& properties = ExecutionProperties{}) {
  engine.run(std::make_unique<xronos::runtime::default_::DefaultRuntime>(), properties);
}

TEST_CASE("A startup reaction executes", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);

  int invocations{0};
  auto reaction = abi_backend.register_reaction(
      "r", reactor, make_callback<xronos::abi::ReactionHandler>([&invocations]() { invocations++; }), 1);
  abi_backend.register_reaction_trigger(reaction, startup);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(invocations == 1);
}

TEST_CASE("Values cross connected ports", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);
  abi_backend.add_connection(output, input);

  // Handlers look their dependencies up through the backend's lookup
  // facade, exactly like the SDK's inline glue does.
  auto& runtime_backend = abi_backend.runtime_backend();
  ElementUid send_uid{};
  ElementUid receive_uid{};
  bool input_was_present{false};
  int received{-1};

  send_uid = abi_backend.register_reaction("send", reactor, make_callback<xronos::abi::ReactionHandler>([&]() {
                                             auto* effect = runtime_backend.get_settable_effect(send_uid, output);
                                             if (effect != nullptr) {
                                               effect->set(xronos::value::make<int>(42));
                                             }
                                           }),
                                           1);
  receive_uid = abi_backend.register_reaction("receive", reactor, make_callback<xronos::abi::ReactionHandler>([&]() {
                                                const auto* trigger = runtime_backend.get_trigger(receive_uid, input);
                                                if (trigger != nullptr && trigger->get().has_value()) {
                                                  input_was_present = true;
                                                  const auto* received_ptr = xronos::value::get_if<int>(trigger->get());
                                                  if (received_ptr != nullptr) {
                                                    received = *received_ptr;
                                                  }
                                                }
                                              }),
                                              2);

  abi_backend.register_reaction_trigger(send_uid, startup);
  abi_backend.register_reaction_effect(send_uid, output);
  abi_backend.register_reaction_trigger(receive_uid, input);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(input_was_present);
  CHECK(received == 42);
}

TEST_CASE("A periodic timer fires until shutdown is triggered", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto timer = abi_backend.register_periodic_timer("timer", reactor, 1ms, 10ms);
  auto shutdown = abi_backend.register_shutdown("shutdown", reactor);

  auto& runtime_backend = abi_backend.runtime_backend();
  ElementUid reaction_uid{};
  int firings{0};

  reaction_uid = abi_backend.register_reaction("count", reactor, make_callback<xronos::abi::ReactionHandler>([&]() {
                                                 firings++;
                                                 if (firings == 3) {
                                                   auto* effect =
                                                       runtime_backend.get_shutdown_effect(reaction_uid, shutdown);
                                                   if (effect != nullptr) {
                                                     effect->trigger_shutdown();
                                                   }
                                                 }
                                               }),
                                               1);
  abi_backend.register_reaction_trigger(reaction_uid, timer);
  abi_backend.register_reaction_effect(reaction_uid, shutdown);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine, ExecutionProperties{.fast_mode = true});

  CHECK(firings == 3);
}

TEST_CASE("Assemble callbacks run in registration order and can register elements", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto first = abi_backend.register_top_level_reactor("first");
  auto second = abi_backend.register_top_level_reactor("second");
  auto third = abi_backend.register_top_level_reactor("third");

  std::vector<std::string> order{};
  int invocations{0};
  abi_backend.register_assemble_callback(
      first, make_callback<xronos::abi::AssembleCallback>([&]() {
        order.emplace_back("first");
        // Callbacks may register further elements, like Reactor::assemble does.
        auto startup = abi_backend.register_startup("startup", first);
        auto reaction = abi_backend.register_reaction(
            "r", first, make_callback<xronos::abi::ReactionHandler>([&invocations]() { invocations++; }), 1);
        abi_backend.register_reaction_trigger(reaction, startup);
      }));
  abi_backend.register_assemble_callback(
      second, make_callback<xronos::abi::AssembleCallback>([&order]() { order.emplace_back("second"); }));
  abi_backend.register_assemble_callback(
      third, make_callback<xronos::abi::AssembleCallback>([&order]() { order.emplace_back("third"); }));
  abi_backend.unregister_assemble_callback(second);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(order == std::vector<std::string>{"first", "third"});
  CHECK(invocations == 1);
}

TEST_CASE("Duplicate names are rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  (void)abi_backend.register_input_port("port", reactor);
  CHECK_THROWS_AS((void)abi_backend.register_input_port("port", reactor), xronos::abi::InvalidNameError);
  // One exception type covers both rejected-name conditions, so the message
  // is what tells a duplicate apart from a malformed name.
  CHECK_THROWS_WITH((void)abi_backend.register_input_port("port", reactor),
                    Catch::Matchers::ContainsSubstring("there already is a"));
}

TEST_CASE("Invalid element names are rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  SECTION("empty") {
    CHECK_THROWS_AS((void)abi_backend.register_input_port("", reactor), xronos::abi::InvalidNameError);
  }
  SECTION("containing a dot") {
    CHECK_THROWS_AS((void)abi_backend.register_input_port("bad.name", reactor), xronos::abi::InvalidNameError);
  }
  SECTION("containing whitespace") {
    CHECK_THROWS_AS((void)abi_backend.register_input_port("bad name", reactor), xronos::abi::InvalidNameError);
  }
  SECTION("containing a reserved syntax character") {
    for (const auto* name : {"bad,name", "bad/name", "bad*name", "bad$name", "bad?name", "bad#name", "bad@name"}) {
      CHECK_THROWS_AS((void)abi_backend.register_input_port(name, reactor), xronos::abi::InvalidNameError);
    }
  }
  SECTION("the message names the rule rather than reporting a duplicate") {
    CHECK_THROWS_WITH((void)abi_backend.register_input_port("bad.name", reactor),
                      Catch::Matchers::ContainsSubstring("must be non-empty and must not contain"));
  }
}

TEST_CASE("A second inbound connection is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto out1 = abi_backend.register_output_port("out1", reactor);
  auto out2 = abi_backend.register_output_port("out2", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  abi_backend.add_connection(out1, input);
  CHECK_THROWS_AS(abi_backend.add_connection(out2, input), xronos::abi::ValidationError);
}

TEST_CASE("A connection that violates the reactor hierarchy is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto grandchild = abi_backend.register_reactor("subsub", child);
  auto input = abi_backend.register_input_port("in", reactor);
  auto grandchild_input = abi_backend.register_input_port("in", grandchild);

  // The connection skips a level of the hierarchy: it may only reach the
  // ports of a direct child.
  CHECK_THROWS_AS(abi_backend.add_connection(input, grandchild_input), xronos::abi::ValidationError);
  CHECK_THROWS_WITH(abi_backend.add_connection(input, grandchild_input),
                    Catch::Matchers::ContainsSubstring("does not respect the reactor hierarchy") &&
                        Catch::Matchers::ContainsSubstring("main.in") &&
                        Catch::Matchers::ContainsSubstring("main.sub.subsub.in"));
}

TEST_CASE("A connection with a non-port endpoint is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto timer = abi_backend.register_periodic_timer("timer", reactor, 1ms, 10ms);
  auto input = abi_backend.register_input_port("in", reactor);

  CHECK_THROWS_AS(abi_backend.add_connection(timer, input), xronos::abi::ValidationError);
  CHECK_THROWS_WITH(abi_backend.add_connection(timer, input), Catch::Matchers::ContainsSubstring("main.timer") &&
                                                                  Catch::Matchers::ContainsSubstring("periodic timer"));
}

TEST_CASE("A trigger that violates the reactor hierarchy is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto grandchild = abi_backend.register_reactor("subsub", child);
  auto reaction = abi_backend.register_reaction("r", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);

  SECTION("a grandchild's output is out of reach") {
    auto grandchild_output = abi_backend.register_output_port("out", grandchild);
    CHECK_THROWS_AS(abi_backend.register_reaction_trigger(reaction, grandchild_output), xronos::abi::ValidationError);
    CHECK_THROWS_WITH(abi_backend.register_reaction_trigger(reaction, grandchild_output),
                      Catch::Matchers::ContainsSubstring("main.r") &&
                          Catch::Matchers::ContainsSubstring("main.sub.subsub.out") &&
                          Catch::Matchers::ContainsSubstring("as a trigger"));
  }
  SECTION("the reactor's own output is not a trigger") {
    auto output = abi_backend.register_output_port("out", reactor);
    CHECK_THROWS_AS(abi_backend.register_reaction_trigger(reaction, output), xronos::abi::ValidationError);
  }
  SECTION("a child's input is not a trigger") {
    auto child_input = abi_backend.register_input_port("in", child);
    CHECK_THROWS_AS(abi_backend.register_reaction_trigger(reaction, child_input), xronos::abi::ValidationError);
  }
}

TEST_CASE("An effect that violates the reactor hierarchy is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto grandchild = abi_backend.register_reactor("subsub", child);
  auto reaction = abi_backend.register_reaction("r", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);

  SECTION("a grandchild's input is out of reach") {
    auto grandchild_input = abi_backend.register_input_port("in", grandchild);
    CHECK_THROWS_AS(abi_backend.register_reaction_effect(reaction, grandchild_input), xronos::abi::ValidationError);
    CHECK_THROWS_WITH(abi_backend.register_reaction_effect(reaction, grandchild_input),
                      Catch::Matchers::ContainsSubstring("main.r") &&
                          Catch::Matchers::ContainsSubstring("main.sub.subsub.in") &&
                          Catch::Matchers::ContainsSubstring("as an effect"));
  }
  SECTION("the reactor's own input is not an effect") {
    auto input = abi_backend.register_input_port("in", reactor);
    CHECK_THROWS_AS(abi_backend.register_reaction_effect(reaction, input), xronos::abi::ValidationError);
  }
  SECTION("a child's output is not an effect") {
    auto child_output = abi_backend.register_output_port("out", child);
    CHECK_THROWS_AS(abi_backend.register_reaction_effect(reaction, child_output), xronos::abi::ValidationError);
  }
}

TEST_CASE("Attributes are only added once per key", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  CHECK(abi_backend.add_attribute(reactor, "key", std::string{"value"}));
  CHECK_FALSE(abi_backend.add_attribute(reactor, "key", std::string{"other"}));
  CHECK(abi_backend.add_attribute(reactor, "flag", true));
  CHECK(abi_backend.add_attribute(reactor, "count", std::int64_t{3}));
  CHECK(abi_backend.add_attribute(reactor, "ratio", 0.5));
}

TEST_CASE("FQNs can be read back", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto outer = abi_backend.register_top_level_reactor("outer");
  auto inner = abi_backend.register_reactor("inner", outer);
  auto port = abi_backend.register_input_port("port", inner);

  CHECK(abi_backend.element_fqn(outer) == "outer");
  CHECK(abi_backend.element_fqn(inner) == "outer.inner");
  CHECK(abi_backend.element_fqn(port) == "outer.inner.port");
}

TEST_CASE("Triggers before a run is prepared are dropped as NotStarted", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto event = abi_backend.register_physical_event("event", reactor);

  // The facade exists from backend creation; it drops trigger attempts
  // until run() publishes the prepared program.
  auto& runtime_backend = abi_backend.runtime_backend();
  CHECK(runtime_backend.trigger_physical_event(event, xronos::value::make<xronos::abi::Void>()) ==
        xronos::abi::TriggerStatus::NotStarted);
  engine.assemble();
  CHECK(runtime_backend.trigger_physical_event(event, xronos::value::make<xronos::abi::Void>()) ==
        xronos::abi::TriggerStatus::NotStarted);
  // With no prepared run there is nothing to resolve a uid against, so a
  // uid that names no physical event also reports NotStarted.
  CHECK(runtime_backend.trigger_physical_event(reactor, xronos::value::make<xronos::abi::Void>()) ==
        xronos::abi::TriggerStatus::NotStarted);
}

TEST_CASE("Concurrent triggers through the facade are safe", "[backend]") {
  constexpr std::size_t contended_thread_count = 4;
  constexpr std::size_t distinct_event_count = 3;
  // The racy interleavings are only a few instructions wide, so a single
  // attempt rarely lands on one. Each repetition builds a fresh engine (a
  // backend runs only once) and races the deliveries again.
  constexpr std::size_t repetitions = 100;

  for (std::size_t repetition = 0; repetition < repetitions; repetition++) {
    CAPTURE(repetition);
    xronos::backend::Engine engine{};
    auto& abi_backend = engine.abi();
    auto reactor = abi_backend.register_top_level_reactor("main");
    auto shutdown = abi_backend.register_shutdown("shutdown", reactor);
    auto& runtime_backend = abi_backend.runtime_backend();

    // One contended event that several threads look up at once, plus a set of
    // distinct events looked up by one thread each.
    auto contended_event = abi_backend.register_physical_event("contended", reactor);
    int contended_firings{0};
    auto contended_reaction = abi_backend.register_reaction(
        "count_contended", reactor,
        make_callback<xronos::abi::ReactionHandler>([&contended_firings]() { contended_firings++; }), 1);
    abi_backend.register_reaction_trigger(contended_reaction, contended_event);

    std::array<ElementUid, distinct_event_count> distinct_events{};
    std::array<int, distinct_event_count> distinct_firings{};
    for (std::size_t index = 0; index < distinct_event_count; index++) {
      distinct_events.at(index) = abi_backend.register_physical_event("event" + std::to_string(index), reactor);
      auto reaction = abi_backend.register_reaction(
          "count" + std::to_string(index), reactor,
          make_callback<xronos::abi::ReactionHandler>([&count = distinct_firings.at(index)]() { count++; }),
          static_cast<std::uint32_t>(index + 2));
      abi_backend.register_reaction_trigger(reaction, distinct_events.at(index));
    }

    // A dedicated physical event ends the run once all workers have fired.
    auto stop_event = abi_backend.register_physical_event("stop", reactor);
    auto stop_uid = std::make_shared<ElementUid>();
    auto stop_reaction = abi_backend.register_reaction(
        "handle_stop", reactor, make_callback<xronos::abi::ReactionHandler>([&runtime_backend, stop_uid, shutdown]() {
          auto* effect = runtime_backend.get_shutdown_effect(*stop_uid, shutdown);
          if (effect != nullptr) {
            effect->trigger_shutdown();
          }
        }),
        static_cast<std::uint32_t>(distinct_event_count + 2));
    *stop_uid = stop_reaction;
    abi_backend.register_reaction_trigger(stop_reaction, stop_event);
    abi_backend.register_reaction_effect(stop_reaction, shutdown);

    // The scheduler opens its live window shortly after run() publishes the
    // lookups; a trigger attempt in between is dropped as not-started. A
    // startup reaction reports the open window so the workers trigger only
    // inside it.
    std::atomic<bool> started{false};
    auto startup = abi_backend.register_startup("startup", reactor);
    auto startup_reaction = abi_backend.register_reaction(
        "report_started", reactor, make_callback<xronos::abi::ReactionHandler>([&started]() { started.store(true); }),
        static_cast<std::uint32_t>(distinct_event_count + 3));
    abi_backend.register_reaction_trigger(startup_reaction, startup);

    engine.assemble();
    REQUIRE(engine.validate().empty());

    std::thread run_thread{[&engine]() { run(engine); }};

    // The main thread waits for the startup reaction, then releases the
    // barrier: every delivery lands inside the live window, and the
    // deliveries race with each other -- both through the facade's gate and
    // through the runtime's concurrent trigger lookup.
    constexpr std::size_t worker_count = contended_thread_count + distinct_event_count;
    std::barrier barrier{static_cast<std::ptrdiff_t>(worker_count + 1)};

    // Workers only record; per the suite's rule, no Catch2 assertions run on
    // worker threads.
    auto fire = [&runtime_backend, &barrier](ElementUid event, xronos::abi::TriggerStatus& slot) {
      barrier.arrive_and_wait();
      slot = runtime_backend.trigger_physical_event(event, xronos::value::make<xronos::abi::Void>());
    };

    std::array<xronos::abi::TriggerStatus, contended_thread_count> contended_results{};
    std::array<xronos::abi::TriggerStatus, distinct_event_count> distinct_results{};
    std::vector<std::thread> workers{};
    workers.reserve(worker_count);
    for (auto& slot : contended_results) {
      workers.emplace_back(fire, contended_event, std::ref(slot));
    }
    for (std::size_t index = 0; index < distinct_event_count; index++) {
      workers.emplace_back(fire, distinct_events.at(index), std::ref(distinct_results.at(index)));
    }

    while (!started.load()) {
      std::this_thread::yield();
    }
    barrier.arrive_and_wait();
    for (auto& worker : workers) {
      worker.join();
    }
    // All worker fires carry earlier physical timestamps than this one, so the
    // scheduler handles them before the stop reaction shuts the run down.
    auto stop_status = runtime_backend.trigger_physical_event(stop_event, xronos::value::make<xronos::abi::Void>());
    run_thread.join();

    CHECK(stop_status == xronos::abi::TriggerStatus::Accepted);
    for (auto status : contended_results) {
      CHECK(status == xronos::abi::TriggerStatus::Accepted);
    }
    for (auto status : distinct_results) {
      CHECK(status == xronos::abi::TriggerStatus::Accepted);
    }
    CHECK(contended_firings == static_cast<int>(contended_thread_count));
    for (int count : distinct_firings) {
      CHECK(count == 1);
    }
  }
}

// Assembles a program whose single physical event triggers a reaction; the
// handler counts its executions and then requests shutdown. The live-window
// tests below drive it from an external thread.
struct StopOnEventProgram {
  xronos::backend::Engine engine{};
  ElementUid reactor{};
  ElementUid event{};
  int executions{0};

  StopOnEventProgram() {
    auto& abi_backend = engine.abi();
    reactor = abi_backend.register_top_level_reactor("main");
    auto shutdown = abi_backend.register_shutdown("shutdown", reactor);
    event = abi_backend.register_physical_event("event", reactor);

    auto reaction_uid = std::make_shared<ElementUid>();
    auto reaction = abi_backend.register_reaction(
        "stop", reactor, make_callback<xronos::abi::ReactionHandler>([this, reaction_uid, shutdown]() {
          executions++;
          auto* effect = engine.abi().runtime_backend().get_shutdown_effect(*reaction_uid, shutdown);
          if (effect != nullptr) {
            effect->trigger_shutdown();
          }
        }),
        1);
    *reaction_uid = reaction;
    abi_backend.register_reaction_trigger(reaction, event);
    abi_backend.register_reaction_effect(reaction, shutdown);
    engine.assemble();
  }
};

// The attempt reports NotStarted until run() publishes the program and the
// scheduler opens its live window; retrying absorbs that span. Gives up
// after five seconds and returns the last status.
auto trigger_until_accepted(xronos::backend::Engine& engine, ElementUid event) -> xronos::abi::TriggerStatus {
  auto& runtime_backend = engine.abi().runtime_backend();
  auto status = runtime_backend.trigger_physical_event(event, xronos::value::make<xronos::abi::Void>());
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (status != xronos::abi::TriggerStatus::Accepted && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
    status = runtime_backend.trigger_physical_event(event, xronos::value::make<xronos::abi::Void>());
  }
  return status;
}

TEST_CASE("A trigger inside the live window is accepted and delivered", "[backend]") {
  StopOnEventProgram program{};
  REQUIRE(program.engine.validate().empty());

  // The timeout only bounds the test if the trigger is never accepted; the
  // accepted trigger ends the run through the shutdown reaction.
  std::thread run_thread{[&program]() { run(program.engine, ExecutionProperties{.timeout = 10s}); }};
  auto status = trigger_until_accepted(program.engine, program.event);
  run_thread.join();

  CHECK(status == xronos::abi::TriggerStatus::Accepted);
  CHECK(program.executions == 1);
}

TEST_CASE("A trigger after the run is dropped and reports Stopped", "[backend]") {
  StopOnEventProgram program{};
  REQUIRE(program.engine.validate().empty());

  std::thread run_thread{[&program]() { run(program.engine, ExecutionProperties{.timeout = 10s}); }};
  auto accepted = trigger_until_accepted(program.engine, program.event);
  run_thread.join();
  REQUIRE(accepted == xronos::abi::TriggerStatus::Accepted);

  // run() has returned, so the facade's gate is retired and reports the
  // program as stopped without touching it.
  auto& runtime_backend = program.engine.abi().runtime_backend();
  CHECK(runtime_backend.trigger_physical_event(program.event, xronos::value::make<xronos::abi::Void>()) ==
        xronos::abi::TriggerStatus::Stopped);
  CHECK(program.executions == 1);
}

TEST_CASE("A trigger on a uid that is no physical event reports UnknownPhysicalEvent", "[backend]") {
  StopOnEventProgram program{};
  REQUIRE(program.engine.validate().empty());

  std::thread run_thread{[&program]() { run(program.engine, ExecutionProperties{.timeout = 10s}); }};
  auto accepted = trigger_until_accepted(program.engine, program.event);

  // The run is live here. One uid names a registered element that is no
  // physical event, the other was never registered; both report the same
  // status.
  auto& runtime_backend = program.engine.abi().runtime_backend();
  auto wrong_kind_status =
      runtime_backend.trigger_physical_event(program.reactor, xronos::value::make<xronos::abi::Void>());
  auto unregistered_status =
      runtime_backend.trigger_physical_event(program.reactor + 1000, xronos::value::make<xronos::abi::Void>());
  run_thread.join();

  REQUIRE(accepted == xronos::abi::TriggerStatus::Accepted);
  CHECK(wrong_kind_status == xronos::abi::TriggerStatus::UnknownPhysicalEvent);
  CHECK(unregistered_status == xronos::abi::TriggerStatus::UnknownPhysicalEvent);
}

// The lookup-based path stays for consumers compiled against ABI 1.0.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_CASE("The ABI 1.0 trigger path still delivers events", "[backend]") {
  StopOnEventProgram program{};
  REQUIRE(program.engine.validate().empty());

  std::atomic<bool> done{false};
  std::thread run_thread{[&program, &done]() {
    run(program.engine, ExecutionProperties{.timeout = 10s});
    done.store(true);
  }};

  // Poll the lookup facade until run() publishes the prepared program, then
  // fire until the shutdown reaction ends the run. The status-less trigger
  // gives no feedback, so fires before the live window opens are simply
  // dropped and the loop fires again.
  xronos::abi::ExternalTrigger* trigger{nullptr};
  while (trigger == nullptr) {
    trigger = program.engine.abi().runtime_backend().get_external_trigger(program.event);
  }
  while (!done.load()) {
    trigger->trigger(xronos::value::make<xronos::abi::Void>());
    std::this_thread::sleep_for(1ms);
  }
  run_thread.join();

  CHECK(program.executions >= 1);
}
#pragma GCC diagnostic pop

TEST_CASE("Triggers racing the end of a run drain before teardown", "[backend]") {
  constexpr std::size_t worker_count = 4;
  // The interesting interleavings -- a delivery in flight while run()
  // retires the gate, and attempts arriving just after -- are only a few
  // instructions wide, so the race is retried many times.
  constexpr std::size_t repetitions = 100;

  for (std::size_t repetition = 0; repetition < repetitions; repetition++) {
    CAPTURE(repetition);

    std::array<bool, worker_count> sane{};
    std::array<bool, worker_count> accepted{};
    std::vector<std::thread> workers{};
    workers.reserve(worker_count);
    {
      StopOnEventProgram program{};
      REQUIRE(program.engine.validate().empty());
      std::barrier barrier{static_cast<std::ptrdiff_t>(worker_count + 1)};

      // Workers hammer the trigger while the main thread lets run() return.
      // The gate retires when run() returns, so every worker observes
      // Stopped and exits before the engine leaves this scope. Workers only
      // record; the checks run after the joins.
      for (std::size_t index = 0; index < worker_count; index++) {
        workers.emplace_back([&program, &barrier, &sane_slot = sane.at(index), &accepted_slot = accepted.at(index)]() {
          barrier.arrive_and_wait();
          bool only_sane_statuses = true;
          while (true) {
            auto status = program.engine.abi().runtime_backend().trigger_physical_event(
                program.event, xronos::value::make<xronos::abi::Void>());
            if (status == xronos::abi::TriggerStatus::Stopped) {
              break;
            }
            if (status == xronos::abi::TriggerStatus::Accepted) {
              accepted_slot = true;
            } else if (status != xronos::abi::TriggerStatus::NotStarted) {
              only_sane_statuses = false;
            }
          }
          sane_slot = only_sane_statuses;
        });
      }

      barrier.arrive_and_wait();
      // The first accepted fire triggers the shutdown reaction, so the run
      // ends while the workers keep hammering.
      run(program.engine, ExecutionProperties{.timeout = 10s});
      for (auto& worker : workers) {
        worker.join();
      }
      workers.clear();
      // The engine is destroyed here, after every worker observed Stopped.
    }

    for (bool only_sane_statuses : sane) {
      CHECK(only_sane_statuses);
    }
    // The run ends through the shutdown reaction, so some worker's fire was
    // accepted; without this the repetition would not have raced a live
    // delivery against the retiring gate at all.
    CHECK(std::ranges::any_of(accepted, [](bool flag) { return flag; }));
  }
}

TEST_CASE("A stop request ends a run promptly and runs the shutdown reactions", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto timer = abi_backend.register_periodic_timer("timer", reactor, 1h, 1h);
  auto shutdown = abi_backend.register_shutdown("shutdown", reactor);

  std::atomic<bool> started{false};
  int timer_firings{0};
  int shutdown_firings{0};
  auto startup_reaction = abi_backend.register_reaction(
      "report_started", reactor, make_callback<xronos::abi::ReactionHandler>([&started]() { started.store(true); }), 1);
  abi_backend.register_reaction_trigger(startup_reaction, startup);
  auto timer_reaction = abi_backend.register_reaction(
      "count_timer", reactor, make_callback<xronos::abi::ReactionHandler>([&timer_firings]() { timer_firings++; }), 2);
  abi_backend.register_reaction_trigger(timer_reaction, timer);
  auto shutdown_reaction = abi_backend.register_reaction(
      "count_shutdown", reactor,
      make_callback<xronos::abi::ReactionHandler>([&shutdown_firings]() { shutdown_firings++; }), 3);
  abi_backend.register_reaction_trigger(shutdown_reaction, shutdown);

  engine.assemble();
  REQUIRE(engine.validate().empty());

  // The hour-long timer means only the stop request can end the run; a lost
  // request hangs the test instead of passing it.
  std::thread run_thread{[&engine]() { run(engine); }};
  while (!started.load()) {
    std::this_thread::yield();
  }
  engine.request_stop();
  run_thread.join();

  CHECK(timer_firings == 0);
  CHECK(shutdown_firings == 1);
  // A stop request after the run returned is a harmless no-op.
  engine.request_stop();
}

TEST_CASE("A stop request ends a run that waits for external events", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto event = abi_backend.register_physical_event("event", reactor);
  auto shutdown = abi_backend.register_shutdown("shutdown", reactor);

  std::atomic<bool> started{false};
  int event_firings{0};
  int shutdown_firings{0};
  auto startup_reaction = abi_backend.register_reaction(
      "report_started", reactor, make_callback<xronos::abi::ReactionHandler>([&started]() { started.store(true); }), 1);
  abi_backend.register_reaction_trigger(startup_reaction, startup);
  auto event_reaction = abi_backend.register_reaction(
      "count_event", reactor, make_callback<xronos::abi::ReactionHandler>([&event_firings]() { event_firings++; }), 2);
  abi_backend.register_reaction_trigger(event_reaction, event);
  auto shutdown_reaction = abi_backend.register_reaction(
      "count_shutdown", reactor,
      make_callback<xronos::abi::ReactionHandler>([&shutdown_firings]() { shutdown_firings++; }), 3);
  abi_backend.register_reaction_trigger(shutdown_reaction, shutdown);

  engine.assemble();
  REQUIRE(engine.validate().empty());

  // With a physical event source and no timeout the scheduler would wait for
  // external events forever, so only the stop request can end the run.
  std::thread run_thread{[&engine]() { run(engine); }};
  while (!started.load()) {
    std::this_thread::yield();
  }
  engine.request_stop();
  run_thread.join();

  CHECK(event_firings == 0);
  CHECK(shutdown_firings == 1);
}

TEST_CASE("A stop requested before the run starts is honored once it starts", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto timer = abi_backend.register_periodic_timer("timer", reactor, 1h, 1h);
  auto shutdown = abi_backend.register_shutdown("shutdown", reactor);

  int timer_firings{0};
  int shutdown_firings{0};
  auto timer_reaction = abi_backend.register_reaction(
      "count_timer", reactor, make_callback<xronos::abi::ReactionHandler>([&timer_firings]() { timer_firings++; }), 1);
  abi_backend.register_reaction_trigger(timer_reaction, timer);
  auto shutdown_reaction = abi_backend.register_reaction(
      "count_shutdown", reactor,
      make_callback<xronos::abi::ReactionHandler>([&shutdown_firings]() { shutdown_firings++; }), 2);
  abi_backend.register_reaction_trigger(shutdown_reaction, shutdown);

  engine.assemble();
  REQUIRE(engine.validate().empty());

  // The request precedes run(), so no handle exists to forward it to: the
  // engine latches it and run() replays it right after publishing the
  // prepared program. The hour-long timer means only the replayed stop can
  // end the run; a dropped request hangs the test instead of passing it.
  engine.request_stop();
  run(engine);

  CHECK(timer_firings == 0);
  CHECK(shutdown_firings == 1);
}

TEST_CASE("A program can only run once per backend", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  (void)abi_backend.register_top_level_reactor("main");
  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);
  CHECK_THROWS_AS(run(engine), std::logic_error);
}

TEST_CASE("Running requires assembly", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  (void)abi_backend.register_top_level_reactor("main");
  // Completing the model must never be a side effect of running.
  CHECK_THROWS_AS(run(engine), std::logic_error);
}

TEST_CASE("Running requires a runtime", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  (void)abi_backend.register_top_level_reactor("main");
  engine.assemble();
  REQUIRE(engine.validate().empty());

  // A null runtime is rejected up front and does not consume the one-shot
  // run, so a corrected call can still proceed.
  CHECK_THROWS_AS(engine.run(nullptr, ExecutionProperties{}), std::invalid_argument);
  run(engine);
}

TEST_CASE("Running requires validation", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  (void)abi_backend.register_top_level_reactor("main");
  engine.assemble();
  // Checking the model must never be a side effect of running.
  CHECK_THROWS_AS(run(engine), std::logic_error);
}

TEST_CASE("Validation requires assembly", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  (void)abi_backend.register_top_level_reactor("main");
  // The checks only make sense on a completed model.
  CHECK_THROWS_AS((void)engine.validate(), std::logic_error);
}

TEST_CASE("A connection added after assembly carries values", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  auto& runtime_backend = abi_backend.runtime_backend();
  ElementUid send_uid{};
  ElementUid receive_uid{};
  int received{-1};

  send_uid = abi_backend.register_reaction("send", reactor, make_callback<xronos::abi::ReactionHandler>([&]() {
                                             auto* effect = runtime_backend.get_settable_effect(send_uid, output);
                                             if (effect != nullptr) {
                                               effect->set(xronos::value::make<int>(42));
                                             }
                                           }),
                                           1);
  receive_uid = abi_backend.register_reaction("receive", reactor, make_callback<xronos::abi::ReactionHandler>([&]() {
                                                const auto* trigger = runtime_backend.get_trigger(receive_uid, input);
                                                if (trigger != nullptr && trigger->get().has_value()) {
                                                  const auto* received_ptr = xronos::value::get_if<int>(trigger->get());
                                                  if (received_ptr != nullptr) {
                                                    received = *received_ptr;
                                                  }
                                                }
                                              }),
                                              2);

  abi_backend.register_reaction_trigger(send_uid, startup);
  abi_backend.register_reaction_effect(send_uid, output);
  abi_backend.register_reaction_trigger(receive_uid, input);

  // A host may keep wiring the completed model: assembly reveals the ports,
  // the connection lands afterwards, and validation checks the result.
  engine.assemble();
  abi_backend.add_connection(output, input);
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(received == 42);
}

TEST_CASE("A model change outdates an earlier validation", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  engine.assemble();
  REQUIRE(engine.validate().empty());

  // The verdict covered the model without this connection, so running now
  // would execute a graph the validator never saw.
  abi_backend.add_connection(output, input);
  CHECK_THROWS_AS(run(engine), std::logic_error);

  // A fresh passing validate covers the change and run proceeds.
  REQUIRE(engine.validate().empty());
  run(engine);
}

TEST_CASE("Validation sees connections added after assembly", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  // The reaction reads `in` and writes `out`; connecting `out` back to `in`
  // closes a dependency cycle.
  auto reaction =
      abi_backend.register_reaction("loop", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  abi_backend.register_reaction_trigger(reaction, input);
  abi_backend.register_reaction_effect(reaction, output);

  engine.assemble();
  abi_backend.add_connection(output, input);

  // The cycle exists only because of the post-assembly connection, so a
  // finding here proves validation checks the model as wired now.
  CHECK_FALSE(engine.validate().empty());
}

TEST_CASE("A zero-delay cycle through nested port access is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto parent = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", parent);
  auto child_input = abi_backend.register_input_port("in", child);
  auto child_output = abi_backend.register_output_port("out", child);

  // The child forwards its input to its output with zero delay, and the
  // parent reaction feeds the output back to the input by direct port
  // access. No connection is involved, yet the cycle is real.
  auto forward =
      abi_backend.register_reaction("forward", child, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  abi_backend.register_reaction_trigger(forward, child_input);
  abi_backend.register_reaction_effect(forward, child_output);
  auto loop_back =
      abi_backend.register_reaction("loop_back", parent, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  abi_backend.register_reaction_trigger(loop_back, child_output);
  abi_backend.register_reaction_effect(loop_back, child_input);

  engine.assemble();
  auto errors = engine.validate();
  REQUIRE_FALSE(errors.empty());
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("dependency cycle"));
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.loop_back"));
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.sub.forward"));
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("direct accesses"));
}

TEST_CASE("An effect on a connected port is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto child = abi_backend.register_reactor("sub", reactor);
  auto child_input = abi_backend.register_input_port("in", child);
  auto input = abi_backend.register_input_port("in", reactor);

  auto reaction =
      abi_backend.register_reaction("write", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  abi_backend.register_reaction_trigger(reaction, startup);

  // The parent both forwards its input to the child's input and writes that
  // input directly. Each declaration respects the hierarchy on its own; only
  // their combination is invalid. The check inspects the assembled model, so
  // the declaration order of the effect and the connection must not matter.
  SECTION("effect declared before the connection") {
    abi_backend.register_reaction_effect(reaction, child_input);
    abi_backend.add_connection(input, child_input);
  }
  SECTION("connection declared before the effect") {
    abi_backend.add_connection(input, child_input);
    abi_backend.register_reaction_effect(reaction, child_input);
  }

  engine.assemble();
  auto errors = engine.validate();
  REQUIRE_FALSE(errors.empty());
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("may not be used as a reaction effect"));
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.write"));
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.sub.in"));
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.in"));
}

TEST_CASE("Connections are rejected once a run is prepared", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);
  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);
  CHECK_THROWS_AS(abi_backend.add_connection(output, input), xronos::abi::ValidationError);
}

TEST_CASE("Trigger declarations are rejected once a run is prepared", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto input = abi_backend.register_input_port("in", reactor);
  auto reaction = abi_backend.register_reaction("r", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);
  CHECK_THROWS_AS(abi_backend.register_reaction_trigger(reaction, input), xronos::abi::ValidationError);
}

TEST_CASE("Effect declarations are rejected once a run is prepared", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("out", reactor);
  auto reaction = abi_backend.register_reaction("r", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);
  CHECK_THROWS_AS(abi_backend.register_reaction_effect(reaction, output), xronos::abi::ValidationError);
}

TEST_CASE("A null reaction handler is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  CHECK_THROWS_AS((void)abi_backend.register_reaction("r", reactor, nullptr, 1), std::invalid_argument);
}

TEST_CASE("Exporting a diagram requires assembly", "[backend]") {
  xronos::backend::Engine engine{};
  CHECK_THROWS_AS(engine.export_diagram(), std::logic_error);
}

TEST_CASE("A backend can only be assembled once", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  (void)abi_backend.register_top_level_reactor("main");
  engine.assemble();
  CHECK_THROWS_AS(engine.assemble(), std::logic_error);
}

// Codecs the tests hand to the backend. The serializer records its own
// destruction, so a test can observe the backend releasing a codec it was
// given.
class TrackingSerializer final : public xronos::abi::PortSerializer {
public:
  explicit TrackingSerializer(bool& destroyed)
      : destroyed_{destroyed} {}
  TrackingSerializer(const TrackingSerializer&) = delete;
  TrackingSerializer(TrackingSerializer&&) = delete;
  auto operator=(const TrackingSerializer&) -> TrackingSerializer& = delete;
  auto operator=(TrackingSerializer&&) -> TrackingSerializer& = delete;
  ~TrackingSerializer() final { destroyed_.get() = true; }

  void serialize(const xronos::abi::AnyValue& /*value*/, xronos::abi::ByteSink& /*sink*/) final {}
  [[nodiscard]] auto deserialize(const std::byte* /*data*/, std::size_t /*size*/) -> xronos::abi::AnyValue final {
    return xronos::value::make<int>(0);
  }

private:
  std::reference_wrapper<bool> destroyed_;
};

// Released into the caller, which hands it to the backend and so takes
// ownership of it.
auto make_serializer(bool& destroyed) -> xronos::abi::PortSerializer* {
  return std::make_unique<TrackingSerializer>(destroyed).release();
}

TEST_CASE("Every model mutation outdates an earlier validation", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);
  auto timer = abi_backend.register_periodic_timer("timer", reactor, 1ms, 10ms);
  auto reaction = abi_backend.register_reaction("r", reactor, make_callback<xronos::abi::ReactionHandler>([]() {}), 1);
  abi_backend.register_reaction_trigger(reaction, timer);

  engine.assemble();
  REQUIRE(engine.validate().empty());

  // One section per kind of model mutation: each must leave run refusing
  // until a fresh validate covers the changed model. A mutator missing its
  // outdating call would slip its section through to a successful run.
  SECTION("registering an element") { (void)abi_backend.register_input_port("late", reactor); }
  SECTION("changing a timer offset") { abi_backend.set_periodic_timer_offset(timer, 2ms); }
  SECTION("changing a timer period") { abi_backend.set_periodic_timer_period(timer, 5ms); }
  SECTION("registering a reaction trigger") { abi_backend.register_reaction_trigger(reaction, input); }
  SECTION("registering a reaction effect") { abi_backend.register_reaction_effect(reaction, output); }
  SECTION("adding a connection") { abi_backend.add_connection(output, input); }
  SECTION("adding a delayed connection") { abi_backend.add_delayed_connection(output, input, 1ms); }
  SECTION("setting a port serializer") {
    bool destroyed{false};
    abi_backend.set_port_serializer(input, make_serializer(destroyed));
  }
  SECTION("exporting a port") {
    bool destroyed{false};
    abi_backend.export_port(reactor, output, "acme.Pose", "json.v1", make_serializer(destroyed));
  }

  CHECK_THROWS_AS(run(engine), std::logic_error);
}

TEST_CASE("Reading the assembled program requires assembly", "[backend]") {
  xronos::backend::Engine engine{};
  // The model does not describe the complete program until the assemble
  // callbacks have run, so all three accessors refuse before then.
  CHECK_THROWS_AS((void)engine.model(), std::logic_error);
  CHECK_THROWS_AS((void)engine.attribute_manager(), std::logic_error);
  CHECK_THROWS_AS((void)engine.source_location_registry(), std::logic_error);
}

// Reads a port's export identity back through the engine's model accessor,
// the way a host inspecting the assembled interface does.
auto export_info_of(const xronos::backend::Engine& engine, ElementUid port)
    -> const std::optional<xronos::core::ExportInfo>& {
  return xronos::core::get_port_properties(engine.model().element_registry.get(port)).export_info;
}

TEST_CASE("An export is read back from the assembled model", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto node = abi_backend.register_top_level_reactor("node");
  auto output = abi_backend.register_output_port("out", node);
  auto internal = abi_backend.register_input_port("internal", node);

  bool unused_destroyed{false};
  abi_backend.export_port(node, output, "acme.Pose", "json.v1", make_serializer(unused_destroyed));
  engine.assemble();

  const auto& info = export_info_of(engine, output);
  REQUIRE(info.has_value());
  CHECK(info->value_type == "acme.Pose");
  CHECK(info->encoding == "json.v1");
  CHECK_FALSE(export_info_of(engine, internal).has_value());
}

TEST_CASE("An export with a null serializer is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto node = abi_backend.register_top_level_reactor("node");
  auto port = abi_backend.register_output_port("out", node);

  // Unlike set_port_serializer, where a null leaves the port unserialized,
  // an export declares an identity the port must be encodable under.
  CHECK_THROWS_AS(abi_backend.export_port(node, port, "acme.Pose", "json.v1", nullptr), std::invalid_argument);

  engine.assemble();
  CHECK_FALSE(export_info_of(engine, port).has_value());
}

TEST_CASE("An export with an empty identity is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto node = abi_backend.register_top_level_reactor("node");
  auto port = abi_backend.register_output_port("out", node);

  // An empty name identifies nothing, so letting one through would make ports
  // carrying unrelated types or encodings compare as a match.
  bool value_type_destroyed{false};
  bool encoding_destroyed{false};
  CHECK_THROWS_AS(abi_backend.export_port(node, port, "", "json.v1", make_serializer(value_type_destroyed)),
                  std::invalid_argument);
  CHECK_THROWS_AS(abi_backend.export_port(node, port, "acme.Pose", "", make_serializer(encoding_destroyed)),
                  std::invalid_argument);

  // Ownership still transferred on both rejected calls.
  CHECK(value_type_destroyed);
  CHECK(encoding_destroyed);

  // Neither attempt left the port exported.
  engine.assemble();
  CHECK_FALSE(export_info_of(engine, port).has_value());
}

TEST_CASE("Exporting a port that is not the node's own is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto node = abi_backend.register_top_level_reactor("node");
  auto child = abi_backend.register_reactor("child", node);
  auto port = abi_backend.register_output_port("out", child);

  bool destroyed{false};
  CHECK_THROWS_AS(abi_backend.export_port(node, port, "acme.Pose", "json.v1", make_serializer(destroyed)),
                  xronos::abi::ValidationError);
  // The backend owns the codecs from the call, even when it rejects the
  // export.
  CHECK(destroyed);
}

TEST_CASE("Exporting the same port twice is rejected", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto node = abi_backend.register_top_level_reactor("node");
  auto port = abi_backend.register_output_port("out", node);

  bool first_destroyed{false};
  bool second_destroyed{false};
  abi_backend.export_port(node, port, "acme.Pose", "json.v1", make_serializer(first_destroyed));
  CHECK_THROWS_AS(abi_backend.export_port(node, port, "acme.Pose", "json.v1", make_serializer(second_destroyed)),
                  xronos::abi::ValidationError);
  CHECK_FALSE(first_destroyed);
  CHECK(second_destroyed);
}

TEST_CASE("An export replaces codecs the port already carries", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto node = abi_backend.register_top_level_reactor("node");
  auto port = abi_backend.register_output_port("out", node);

  bool earlier_destroyed{false};
  bool export_destroyed{false};
  abi_backend.set_port_serializer(port, make_serializer(earlier_destroyed));
  CHECK_FALSE(earlier_destroyed);

  abi_backend.export_port(node, port, "acme.Pose", "json.v1", make_serializer(export_destroyed));
  // The export supersedes what the port carried: the earlier serializer is
  // released, the exported one remains.
  CHECK(earlier_destroyed);
  CHECK_FALSE(export_destroyed);
}

// Counts calls per direction, so a test can see which codec handled a value.
struct CodecCounters {
  int serialized{0};
  int deserialized{0};
};

// An honest little-endian int codec. `on_deserialize` transforms the decoded
// value, giving each codec an observable signature in the boxes it creates.
class IntCodec final : public xronos::abi::PortSerializer {
public:
  explicit IntCodec(CodecCounters& counters, std::function<int(int)> on_deserialize = {})
      : counters_{counters}
      , on_deserialize_{std::move(on_deserialize)} {}

  void serialize(const xronos::abi::AnyValue& value, xronos::abi::ByteSink& sink) final {
    counters_.get().serialized++;
    const auto* int_value = xronos::value::get_if<int>(value);
    if (int_value == nullptr) {
      throw std::runtime_error{"expected an int value"};
    }
    auto bits = static_cast<std::uint32_t>(*int_value);
    std::array<std::byte, 4> bytes{};
    for (std::size_t index = 0; index < bytes.size(); index++) {
      bytes.at(index) = static_cast<std::byte>((bits >> (8U * index)) & 0xFFU);
    }
    sink.write(bytes.data(), bytes.size());
  }

  [[nodiscard]] auto deserialize(const std::byte* data, std::size_t size) -> xronos::abi::AnyValue final {
    counters_.get().deserialized++;
    if (size != 4) {
      throw std::runtime_error{"expected four bytes"};
    }
    auto bytes = std::span{data, size};
    std::uint32_t bits{0};
    for (std::size_t index = 0; index < bytes.size(); index++) {
      bits |= std::to_integer<std::uint32_t>(bytes[index]) << (8U * index);
    }
    auto decoded = static_cast<int>(bits);
    return xronos::value::make<int>(on_deserialize_ ? on_deserialize_(decoded) : decoded);
  }

private:
  std::reference_wrapper<CodecCounters> counters_;
  std::function<int(int)> on_deserialize_;
};

// A codec for valueless events: it round-trips zero bytes.
class VoidCodec final : public xronos::abi::PortSerializer {
public:
  explicit VoidCodec(CodecCounters& counters)
      : counters_{counters} {}

  void serialize(const xronos::abi::AnyValue& /*value*/, xronos::abi::ByteSink& /*sink*/) final {
    counters_.get().serialized++;
  }
  [[nodiscard]] auto deserialize(const std::byte* /*data*/, std::size_t size) -> xronos::abi::AnyValue final {
    counters_.get().deserialized++;
    if (size != 0) {
      throw std::runtime_error{"expected zero bytes"};
    }
    return xronos::value::make<xronos::abi::Void>();
  }

private:
  std::reference_wrapper<CodecCounters> counters_;
};

// Serializes nothing and refuses to deserialize.
class ThrowingDeserializerCodec final : public xronos::abi::PortSerializer {
public:
  void serialize(const xronos::abi::AnyValue& /*value*/, xronos::abi::ByteSink& /*sink*/) final {}
  [[nodiscard]] auto deserialize(const std::byte* /*data*/, std::size_t /*size*/) -> xronos::abi::AnyValue final {
    throw std::runtime_error{"deliberate deserialization failure"};
  }
};

// Released into the caller, which hands the codec to the backend and so
// transfers ownership of it.
template <class Codec, class... Args> auto make_codec(Args&&... args) -> xronos::abi::PortSerializer* {
  return std::make_unique<Codec>(std::forward<Args>(args)...).release();
}

// Registers a reaction that sets `port` to `value` on startup.
auto register_int_send(xronos::abi::Backend& abi_backend, ElementUid reactor, const std::string& name,
                       ElementUid startup, ElementUid port, std::uint32_t position, int value) -> ElementUid {
  auto send_uid = std::make_shared<ElementUid>();
  auto reaction = abi_backend.register_reaction(
      name, reactor, make_callback<xronos::abi::ReactionHandler>([&abi_backend, send_uid, port, value]() {
        auto* effect = abi_backend.runtime_backend().get_settable_effect(*send_uid, port);
        if (effect != nullptr) {
          effect->set(xronos::value::make<int>(value));
        }
      }),
      position);
  *send_uid = reaction;
  abi_backend.register_reaction_trigger(reaction, startup);
  abi_backend.register_reaction_effect(reaction, port);
  return reaction;
}

// Registers a reaction triggered by `port` that records the int arriving
// there into `received`.
auto register_int_probe(xronos::abi::Backend& abi_backend, ElementUid reactor, const std::string& name, ElementUid port,
                        std::uint32_t position, int& received) -> ElementUid {
  auto probe_uid = std::make_shared<ElementUid>();
  auto reaction = abi_backend.register_reaction(
      name, reactor, make_callback<xronos::abi::ReactionHandler>([&abi_backend, probe_uid, port, &received]() {
        const auto* trigger = abi_backend.runtime_backend().get_trigger(*probe_uid, port);
        if (trigger != nullptr && trigger->get().has_value()) {
          const auto* value = xronos::value::get_if<int>(trigger->get());
          if (value != nullptr) {
            received = *value;
          }
        }
      }),
      position);
  *probe_uid = reaction;
  abi_backend.register_reaction_trigger(reaction, port);
  return reaction;
}

TEST_CASE("A cross-boundary connection without a serializer fails validation", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  CodecCounters producer{};
  abi_backend.set_port_serializer(output, make_codec<IntCodec>(producer));
  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Both);

  engine.assemble();
  auto errors = engine.validate();
  REQUIRE(errors.size() == 1);
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.in") &&
                                 Catch::Matchers::ContainsSubstring("cross-boundary connection") &&
                                 Catch::Matchers::ContainsSubstring("no serializer"));
}

TEST_CASE("A cross-boundary connection round-trips values through both codecs", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  CodecCounters producer{};
  CodecCounters consumer{};
  abi_backend.set_port_serializer(output, make_codec<IntCodec>(producer));
  // The consumer's transform proves the delivered box was created by the
  // consumer's codec and not passed through directly.
  abi_backend.set_port_serializer(input, make_codec<IntCodec>(consumer, [](int value) { return value + 1000; }));
  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Both);

  (void)register_int_send(abi_backend, reactor, "send", startup, output, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", input, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(received == 1042);
  CHECK(producer.serialized == 1);
  CHECK(producer.deserialized == 0);
  CHECK(consumer.serialized == 0);
  CHECK(consumer.deserialized == 1);
}

TEST_CASE("A delayed cross-boundary connection delivers the round-tripped value", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  CodecCounters producer{};
  CodecCounters consumer{};
  abi_backend.set_port_serializer(output, make_codec<IntCodec>(producer));
  abi_backend.set_port_serializer(input, make_codec<IntCodec>(consumer, [](int value) { return value + 1000; }));
  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Both, 1ms);

  (void)register_int_send(abi_backend, reactor, "send", startup, output, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", input, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine, ExecutionProperties{.fast_mode = true});

  CHECK(received == 1042);
  CHECK(producer.serialized == 1);
  CHECK(consumer.deserialized == 1);
}

TEST_CASE("A fan-out applies serialization per edge", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto first = abi_backend.register_input_port("first", reactor);
  auto second = abi_backend.register_input_port("second", reactor);
  auto plain = abi_backend.register_input_port("plain", reactor);

  CodecCounters producer{};
  CodecCounters first_consumer{};
  CodecCounters second_consumer{};
  abi_backend.set_port_serializer(output, make_codec<IntCodec>(producer));
  abi_backend.set_port_serializer(first, make_codec<IntCodec>(first_consumer, [](int value) { return value + 1000; }));
  abi_backend.set_port_serializer(second,
                                  make_codec<IntCodec>(second_consumer, [](int value) { return value + 2000; }));
  engine.add_cross_boundary_connection(output, first, BoundaryCrossing::Both);
  engine.add_cross_boundary_connection(output, second, BoundaryCrossing::Both);
  abi_backend.add_connection(output, plain);

  (void)register_int_send(abi_backend, reactor, "send", startup, output, 1, 42);
  int at_first{-1};
  int at_second{-1};
  int at_plain{-1};
  (void)register_int_probe(abi_backend, reactor, "receive_first", first, 2, at_first);
  (void)register_int_probe(abi_backend, reactor, "receive_second", second, 3, at_second);
  (void)register_int_probe(abi_backend, reactor, "receive_plain", plain, 4, at_plain);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  // The differing observed values prove each marked edge ran its own round
  // trip while the unmarked edge passed the box through directly.
  CHECK(at_first == 1042);
  CHECK(at_second == 2042);
  CHECK(at_plain == 42);
  // Both marked edges leave the same port and so share one serialization
  // node, so the fan-out serialized the value once.
  CHECK(producer.serialized == 1);
  CHECK(first_consumer.deserialized == 1);
  CHECK(second_consumer.deserialized == 1);
}

TEST_CASE("A void cross-boundary connection round-trips zero bytes", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  CodecCounters producer{};
  CodecCounters consumer{};
  abi_backend.set_port_serializer(output, make_codec<VoidCodec>(producer));
  abi_backend.set_port_serializer(input, make_codec<VoidCodec>(consumer));
  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Both);

  auto send_uid = std::make_shared<ElementUid>();
  auto send = abi_backend.register_reaction(
      "send", reactor, make_callback<xronos::abi::ReactionHandler>([&abi_backend, send_uid, output]() {
        auto* effect = abi_backend.runtime_backend().get_settable_effect(*send_uid, output);
        if (effect != nullptr) {
          effect->set(xronos::value::make<xronos::abi::Void>());
        }
      }),
      1);
  *send_uid = send;
  abi_backend.register_reaction_trigger(send, startup);
  abi_backend.register_reaction_effect(send, output);

  auto receive_uid = std::make_shared<ElementUid>();
  bool fired{false};
  auto receive = abi_backend.register_reaction(
      "receive", reactor, make_callback<xronos::abi::ReactionHandler>([&abi_backend, receive_uid, input, &fired]() {
        const auto* trigger = abi_backend.runtime_backend().get_trigger(*receive_uid, input);
        if (trigger != nullptr && trigger->get().has_value()) {
          fired = true;
        }
      }),
      2);
  *receive_uid = receive;
  abi_backend.register_reaction_trigger(receive, input);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(fired);
  CHECK(producer.serialized == 1);
  CHECK(consumer.deserialized == 1);
}

TEST_CASE("A cross-boundary connection forwarded on the consumer side re-encodes at the boundary", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto startup = abi_backend.register_startup("startup", child);
  auto source = abi_backend.register_output_port("source", child);
  auto boundary = abi_backend.register_output_port("boundary", reactor);
  auto sink = abi_backend.register_input_port("sink", reactor);

  CodecCounters producer{};
  CodecCounters consumer{};
  abi_backend.set_port_serializer(source, make_codec<IntCodec>(producer));
  abi_backend.set_port_serializer(boundary, make_codec<IntCodec>(consumer, [](int value) { return value + 7; }));
  // The boundary crossing sits between the child's source and the parent's
  // boundary port; the plain connection forwards onwards to a port with no
  // serializer at all.
  engine.add_cross_boundary_connection(source, boundary, BoundaryCrossing::Both);
  abi_backend.add_connection(boundary, sink);

  (void)register_int_send(abi_backend, child, "send", startup, source, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", sink, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  // The value observed behind the boundary carries the boundary codec's
  // signature: it was round-tripped through source's serializer and
  // boundary's deserializer before the plain delivery to sink.
  CHECK(received == 49);
  CHECK(producer.serialized == 1);
  CHECK(consumer.deserialized == 1);
}

TEST_CASE("A cross-boundary connection forwarded on the producer side re-encodes at the boundary", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto startup = abi_backend.register_startup("startup", child);
  auto source = abi_backend.register_output_port("source", child);
  auto boundary = abi_backend.register_output_port("boundary", reactor);
  auto sink = abi_backend.register_input_port("sink", reactor);

  CodecCounters producer{};
  CodecCounters consumer{};
  // The plain connection forwards the child source's box to the parent's
  // boundary port; only the boundary and the sink carry serializers.
  abi_backend.set_port_serializer(boundary, make_codec<IntCodec>(producer));
  abi_backend.set_port_serializer(sink, make_codec<IntCodec>(consumer, [](int value) { return value + 7; }));
  abi_backend.add_connection(source, boundary);
  engine.add_cross_boundary_connection(boundary, sink, BoundaryCrossing::Both);

  (void)register_int_send(abi_backend, child, "send", startup, source, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", sink, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(received == 49);
  CHECK(producer.serialized == 1);
  CHECK(consumer.deserialized == 1);
}

TEST_CASE("A pass-through chain re-encodes per boundary in order", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto startup = abi_backend.register_startup("startup", child);
  auto first = abi_backend.register_output_port("first", child);
  auto middle = abi_backend.register_output_port("middle", reactor);
  auto last = abi_backend.register_input_port("last", reactor);

  CodecCounters first_codec{};
  CodecCounters middle_codec{};
  CodecCounters last_codec{};
  abi_backend.set_port_serializer(first, make_codec<IntCodec>(first_codec));
  // Doubling then adding do not commute, so the observed value proves the
  // boundaries ran in producer to consumer order.
  abi_backend.set_port_serializer(middle, make_codec<IntCodec>(middle_codec, [](int value) { return value * 2; }));
  abi_backend.set_port_serializer(last, make_codec<IntCodec>(last_codec, [](int value) { return value + 100; }));
  engine.add_cross_boundary_connection(first, middle, BoundaryCrossing::Both);
  engine.add_cross_boundary_connection(middle, last, BoundaryCrossing::Both);

  (void)register_int_send(abi_backend, child, "send", startup, first, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", last, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  // 42 decoded as 84 at the middle boundary, then re-encoded and decoded as
  // 184 at the last. The reversed order would produce 284.
  CHECK(received == 184);
  CHECK(first_codec.serialized == 1);
  CHECK(middle_codec.deserialized == 1);
  CHECK(middle_codec.serialized == 1);
  CHECK(last_codec.deserialized == 1);
}

TEST_CASE("An exit and entry pair round-trips through a serializer-less relay port", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto startup = abi_backend.register_startup("startup", child);
  auto output = abi_backend.register_output_port("out", child);
  auto relay = abi_backend.register_output_port("relay", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  CodecCounters producer{};
  CodecCounters consumer{};
  abi_backend.set_port_serializer(output, make_codec<IntCodec>(producer));
  abi_backend.set_port_serializer(input, make_codec<IntCodec>(consumer, [](int value) { return value + 1000; }));
  // The relay carries no serializer. Inside the region the value exists
  // only in serialized form and merely passes through.
  engine.add_cross_boundary_connection(output, relay, BoundaryCrossing::Exit);
  engine.add_cross_boundary_connection(relay, input, BoundaryCrossing::Entry);

  (void)register_int_send(abi_backend, child, "send", startup, output, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", input, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  run(engine);

  CHECK(received == 1042);
  // One serialization at the exit and one deserialization at the entry.
  // The relay in between re-encodes nothing.
  CHECK(producer.serialized == 1);
  CHECK(producer.deserialized == 0);
  CHECK(consumer.serialized == 0);
  CHECK(consumer.deserialized == 1);
}

TEST_CASE("An entry without an upstream exit fails validation on a live chain", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Entry);
  // The chain is live: a reaction writes the origin port.
  (void)register_int_send(abi_backend, reactor, "send", startup, output, 1, 42);

  engine.assemble();
  auto errors = engine.validate();
  REQUIRE(errors.size() == 1);
  CHECK_THAT(errors.front(), Catch::Matchers::ContainsSubstring("main.out") &&
                                 Catch::Matchers::ContainsSubstring("main.in") &&
                                 Catch::Matchers::ContainsSubstring("boundary entry") &&
                                 Catch::Matchers::ContainsSubstring("no upstream"));
}

TEST_CASE("A stray entry on a dead chain passes validation", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto output = abi_backend.register_output_port("relay", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  // No reaction writes the origin, so nothing ever flows here and the
  // stray mark is inert.
  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Entry);

  engine.assemble();
  CHECK(engine.validate().empty());
}

TEST_CASE("An exit and entry pair requires serializers only at its exit and entry ports", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto child = abi_backend.register_reactor("sub", reactor);
  auto output = abi_backend.register_output_port("out", child);
  auto relay = abi_backend.register_output_port("relay", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  engine.add_cross_boundary_connection(output, relay, BoundaryCrossing::Exit);
  engine.add_cross_boundary_connection(relay, input, BoundaryCrossing::Entry);

  engine.assemble();
  auto errors = engine.validate();
  // The serializer check applies even though nothing writes the chain,
  // because assembly resolves serializers eagerly. Only the exit's from
  // port and the entry's to port need one. The relay passes serialized
  // bytes through and stays out of the errors.
  REQUIRE(errors.size() == 2);
  CHECK_THAT(errors[0],
             Catch::Matchers::ContainsSubstring("main.sub.out") && !Catch::Matchers::ContainsSubstring("relay"));
  CHECK_THAT(errors[1], Catch::Matchers::ContainsSubstring("main.in") && !Catch::Matchers::ContainsSubstring("relay"));
}

TEST_CASE("A throwing deserializer fails the run and rethrows the exception", "[backend]") {
  xronos::backend::Engine engine{};
  auto& abi_backend = engine.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);
  auto output = abi_backend.register_output_port("out", reactor);
  auto input = abi_backend.register_input_port("in", reactor);

  CodecCounters producer{};
  abi_backend.set_port_serializer(output, make_codec<IntCodec>(producer));
  abi_backend.set_port_serializer(input, make_codec<ThrowingDeserializerCodec>());
  engine.add_cross_boundary_connection(output, input, BoundaryCrossing::Both);

  (void)register_int_send(abi_backend, reactor, "send", startup, output, 1, 42);
  int received{-1};
  (void)register_int_probe(abi_backend, reactor, "receive", input, 2, received);

  engine.assemble();
  REQUIRE(engine.validate().empty());
  CHECK_THROWS_WITH(run(engine), Catch::Matchers::Equals("deliberate deserialization failure"));
  CHECK(received == -1);
}

// The SDK's `backend::Backend` composes the engine with a runtime factory.
// These tests cover what it adds on top: the factory drives the run, and the
// ABI view survives moves of the handle.

auto make_default_runtime_factory(bool& invoked) -> xronos::backend::RuntimeFactory {
  return [&invoked]() {
    invoked = true;
    return std::make_unique<xronos::runtime::default_::DefaultRuntime>();
  };
}

TEST_CASE("The SDK backend runs on the runtime its factory produces", "[backend]") {
  bool factory_invoked{false};
  xronos::backend::Backend backend{make_default_runtime_factory(factory_invoked)};
  auto& abi_backend = backend.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");
  auto startup = abi_backend.register_startup("startup", reactor);

  int invocations{0};
  auto reaction = abi_backend.register_reaction(
      "r", reactor, make_callback<xronos::abi::ReactionHandler>([&invocations]() { invocations++; }), 1);
  abi_backend.register_reaction_trigger(reaction, startup);

  backend.assemble();
  REQUIRE(backend.validate().empty());
  CHECK_FALSE(factory_invoked);
  backend.run(ExecutionProperties{});

  CHECK(factory_invoked);
  CHECK(invocations == 1);
}

TEST_CASE("Replacing the runtime factory changes the runtime the run uses", "[backend]") {
  bool original_invoked{false};
  bool replacement_invoked{false};
  xronos::backend::Backend backend{make_default_runtime_factory(original_invoked)};
  backend.set_runtime_factory(make_default_runtime_factory(replacement_invoked));

  (void)backend.abi().register_top_level_reactor("main");
  backend.assemble();
  REQUIRE(backend.validate().empty());
  backend.run(ExecutionProperties{});

  CHECK_FALSE(original_invoked);
  CHECK(replacement_invoked);
}

TEST_CASE("The ABI view stays valid across moves of the SDK backend", "[backend]") {
  bool factory_invoked{false};
  xronos::backend::Backend original{make_default_runtime_factory(factory_invoked)};
  auto& abi_backend = original.abi();
  auto reactor = abi_backend.register_top_level_reactor("main");

  xronos::backend::Backend moved{std::move(original)};
  // The reference obtained before the move still addresses the same engine
  // the moved-to backend now owns.
  CHECK(&moved.abi() == &abi_backend);
  CHECK(abi_backend.element_fqn(reactor) == "main");
}

} // namespace
