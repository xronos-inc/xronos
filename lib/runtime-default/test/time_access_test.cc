// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// These tests drive the default runtime through the runtime interfaces on a
// hand-built reactor model, the way the engine does. No value is checked
// inside a reaction handler. Handlers only record, and the checks run after
// the run returns.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/runtime/default/default_runtime.hh"
#include "xronos/runtime/interfaces.hh"
#include "gtest/gtest.h"

namespace {

using namespace std::chrono_literals;
using xronos::core::TimePoint;

// Bridges plain callables to the ABI's owned reaction-handler interface.
class FunctionReactionHandler final : public xronos::abi::ReactionHandler {
public:
  explicit FunctionReactionHandler(std::function<void()> handler)
      : handler_{std::move(handler)} {}
  void invoke() final { handler_(); }

private:
  std::function<void()> handler_;
};

// A model with one reactor whose startup reaction runs `on_startup`.
struct StartupProgram {
  xronos::core::ReactorModel model{};
  std::uint64_t reactor_uid{};

  explicit StartupProgram(std::function<void()> on_startup) {
    const xronos::core::Element& reactor =
        model.element_registry.add_new_element("main", xronos::core::ReactorTag{}, std::nullopt).value();
    reactor_uid = reactor.uid;
    const xronos::core::Element& startup =
        model.element_registry.add_new_element("startup", xronos::core::StartupTag{}, reactor.uid).value();
    const xronos::core::Element& reaction =
        model.element_registry
            .add_new_element(
                "on_startup",
                xronos::core::ReactionTag{
                    std::make_unique<xronos::core::ReactionProperties>(xronos::core::ReactionProperties{
                        .handler = std::make_unique<FunctionReactionHandler>(std::move(on_startup)), .position = 0})},
                reactor.uid)
            .value();
    model.reaction_dependency_registry.register_reaction_trigger(reaction.uid, startup.uid);
  }
};

TEST(DefaultRuntimeTest, TimeAccessIsNullUntilTheRunStarts) {
  xronos::runtime::ProgramHandle* handle_in_handler{nullptr};
  std::uint64_t reactor_uid{};
  bool seen_in_handler{false};
  TimePoint start_timestamp_in_handler{};
  TimePoint timestamp_in_handler{};
  StartupProgram program{[&] {
    const auto* time_access = handle_in_handler->get_time_access(reactor_uid);
    seen_in_handler = time_access != nullptr;
    if (time_access != nullptr) {
      start_timestamp_in_handler = time_access->get_start_timestamp();
      timestamp_in_handler = time_access->get_timestamp();
    }
  }};
  reactor_uid = program.reactor_uid;

  xronos::runtime::default_::DefaultRuntime runtime{};
  auto handle = runtime.initialize_reactor_program(program.model, {.fast_mode = true});
  handle_in_handler = handle.get();

  // The start tag does not exist before execute() determines it.
  EXPECT_EQ(handle->get_time_access(reactor_uid), nullptr);

  handle->execute();

  EXPECT_TRUE(seen_in_handler);
  EXPECT_NE(start_timestamp_in_handler, TimePoint{});
  EXPECT_EQ(timestamp_in_handler, start_timestamp_in_handler);
  // The pointer stays valid after the run, until the handle is destroyed.
  const auto* time_access = handle->get_time_access(reactor_uid);
  ASSERT_NE(time_access, nullptr);
  EXPECT_EQ(time_access->get_start_timestamp(), start_timestamp_in_handler);
}

TEST(DefaultRuntimeTest, TimeAccessLookupsFromAnotherThreadAreSafeWhileTheRunStarts) {
  // The window between initialization and the start of the run is a few
  // instructions wide, so each repetition builds a fresh program and races
  // a lookup against execute() again. This suite runs without the
  // ThreadSanitizer, so a green run proves the contract, not the absence of
  // a data race.
  constexpr int repetitions = 100;

  for (int repetition = 0; repetition < repetitions; repetition++) {
    StartupProgram program{[] {}};
    xronos::runtime::default_::DefaultRuntime runtime{};
    auto handle = runtime.initialize_reactor_program(program.model, {.fast_mode = true});

    // The poll ends once execute() has returned, so a regression that never
    // sets the flag fails the test instead of hanging it. The flag stays set
    // after the run ends, so a null from the lookup after that is a
    // regression, not a lost race.
    std::atomic<bool> run_ended{false};
    std::atomic<bool> saw_time_access{false};
    std::atomic<bool> saw_run_values{false};
    std::thread poller{[&] {
      const xronos::runtime::TimeAccess* time_access{nullptr};
      while (time_access == nullptr && !run_ended.load(std::memory_order_acquire)) {
        time_access = handle->get_time_access(program.reactor_uid);
      }
      if (time_access == nullptr) {
        time_access = handle->get_time_access(program.reactor_uid);
      }
      if (time_access == nullptr) {
        return;
      }
      saw_time_access.store(true, std::memory_order_relaxed);
      // Only the start timestamp is read: the current tag advances on the
      // scheduler thread while this thread runs.
      saw_run_values.store(time_access->get_start_timestamp() != TimePoint{}, std::memory_order_relaxed);
    }};

    handle->execute();
    run_ended.store(true, std::memory_order_release);
    poller.join();

    ASSERT_TRUE(saw_time_access.load(std::memory_order_relaxed));
    EXPECT_TRUE(saw_run_values.load(std::memory_order_relaxed));
  }
}

} // namespace
