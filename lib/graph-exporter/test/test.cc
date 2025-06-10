// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <google/protobuf/empty.pb.h>
#include <grpcpp/completion_queue.h>
#include <grpcpp/server_context.h>
#include <grpcpp/support/status.h>
#include <initializer_list>
#include <optional>
#include <stdexcept>

#include "xronos/graph_exporter/detail/exporter.hh"
#include "xronos/graph_exporter/exporter.hh"
#include "xronos/messages/reactor_graph.pb.h"
#include "xronos/runtime/action.hh"
#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/port.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/runtime/reactor.hh"
#include "xronos/services/diagram_generator.grpc.pb.h"

#include <google/protobuf/util/time_util.h>
#include <grpcpp/server_builder.h>

using namespace std::literals::chrono_literals;
using namespace xronos::messages;
using namespace xronos::services;
using namespace xronos::runtime;
using namespace xronos::graph_exporter;

struct EmptyReactor : public Reactor {
  EmptyReactor(const std::string& name, Environment& environment)
      : Reactor(name, environment) {}
  EmptyReactor(const std::string& name, Reactor& container)
      : Reactor(name, container) {}

  void assemble() override {}
};

struct ElementsReactor : public Reactor {
  ElementsReactor(const std::string& name, Environment& environment)
      : Reactor(name, environment) {}
  ElementsReactor(const std::string& name, Reactor& container)
      : Reactor(name, container) {}

  StartupTrigger startup{"startup", *this};
  ShutdownTrigger shutdown{"shutdown", *this};
  Timer timer1{"timer1", *this, 1s, 2s};
  Timer timer2{"timer2", *this, 12ms, 42us};
  LogicalAction<void> laction{"laction", *this};
  PhysicalAction<int> paction{"paction", *this};
  Input<void> input{"input", *this};
  Output<float> output{"output", *this};
  Reaction reaction{"reaction", 1, *this, []() {}};
  Reaction reaction3{"reaction", 3, *this, []() {}};
  EmptyReactor empty{"empty", *this};

  void assemble() override {
    reaction3.set_deadline(37ms, []() {});
  }
};

struct SrcReactor : public Reactor {
  SrcReactor(const std::string& name, Environment& environment)
      : Reactor(name, environment) {}
  SrcReactor(const std::string& name, Reactor& container)
      : Reactor(name, container) {}

  Output<void> output{"output", *this};

  void assemble() override {}
};

struct SinkReactor : public Reactor {
  SinkReactor(const std::string& name, Environment& environment)
      : Reactor(name, environment) {}
  SinkReactor(const std::string& name, Reactor& container)
      : Reactor(name, container) {}

  Input<void> input{"input", *this};

  void assemble() override {}
};

auto lookup_element(const reactor_graph::Graph& graph, std::uint64_t uid) -> const reactor_graph::ReactorElement& {
  for (const auto& elem : graph.elements()) {
    if (elem.uid() == uid) {
      return elem;
    }
  }
  throw std::runtime_error("element not found");
}

auto lookup_containment(const reactor_graph::Graph& graph, std::uint64_t uid) -> const reactor_graph::Containment& {
  for (const auto& elem : graph.containments()) {
    if (elem.container_uid() == uid) {
      return elem;
    }
  }
  throw std::runtime_error("containment not found");
}

auto lookup_dependencies(const reactor_graph::Graph& graph,
                         std::uint64_t uid) -> const reactor_graph::ReactionDependencies& {
  for (const auto& elem : graph.dependencies()) {
    if (elem.reaction_uid() == uid) {
      return elem;
    }
  }
  throw std::runtime_error("dependencies not found");
}

void check_reactor_element(const reactor_graph::ReactorElement& elem, const Reactor& reactor) {
  REQUIRE(elem.uid() == reactor.uid());
  REQUIRE(elem.name() == reactor.name());
  REQUIRE(elem.has_reactor());
}

void check_timer_element(const reactor_graph::ReactorElement& elem, const Timer& timer) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  REQUIRE(elem.uid() == timer.uid());
  REQUIRE(elem.name() == timer.name());
  REQUIRE(elem.has_timer());
  const auto& elem_timer = elem.timer();
  REQUIRE(elem_timer.timer_type() == reactor_graph::TimerType::TIMER_TYPE_GENERIC);
  REQUIRE(TimeUtil::DurationToNanoseconds(elem_timer.offset()) == timer.offset().count());
  REQUIRE(TimeUtil::DurationToNanoseconds(elem_timer.period()) == timer.period().count());
}

void check_startup_element(const reactor_graph::ReactorElement& elem, const StartupTrigger& startup) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  REQUIRE(elem.uid() == startup.uid());
  REQUIRE(elem.name() == startup.name());
  REQUIRE(elem.has_timer());
  const auto& elem_timer = elem.timer();
  REQUIRE(elem_timer.timer_type() == reactor_graph::TimerType::TIMER_TYPE_STARTUP);
}

void check_shutdown_element(const reactor_graph::ReactorElement& elem, const ShutdownTrigger& shutdown) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  REQUIRE(elem.uid() == shutdown.uid());
  REQUIRE(elem.name() == shutdown.name());
  REQUIRE(elem.has_timer());
  const auto& elem_timer = elem.timer();
  REQUIRE(elem_timer.timer_type() == reactor_graph::TimerType::TIMER_TYPE_SHUTDOWN);
}

void check_action_element(const reactor_graph::ReactorElement& elem, const BaseAction& action,
                          reactor_graph::ActionType expected_action_type) {
  REQUIRE(elem.uid() == action.uid());
  REQUIRE(elem.name() == action.name());
  REQUIRE(elem.has_action());
  const auto& elem_action = elem.action();
  REQUIRE(elem_action.action_type() == expected_action_type);
}

void check_port_element(const reactor_graph::ReactorElement& elem, const BasePort& port,
                        reactor_graph::PortType expected_port_type) {
  REQUIRE(elem.uid() == port.uid());
  REQUIRE(elem.name() == port.name());
  REQUIRE(elem.has_port());
  const auto& elem_port = elem.port();
  REQUIRE(elem_port.port_type() == expected_port_type);
}

void check_reaction_element(const reactor_graph::ReactorElement& elem, const Reaction& reaction) {
  REQUIRE(elem.uid() == reaction.uid());
  REQUIRE(elem.name() == reaction.name());
  REQUIRE(elem.has_reaction());
  const auto& elem_reaction = elem.reaction();
  REQUIRE(elem_reaction.priority() == reaction.priority());
  REQUIRE(elem_reaction.has_deadline() == reaction.has_deadline());
  if (reaction.has_deadline()) {
    using TimeUtil = google::protobuf::util::TimeUtil;
    REQUIRE(TimeUtil::DurationToNanoseconds(elem_reaction.deadline()) == reaction.deadline().count());
  }
}

TEST_CASE("Serialization of a single empty reactor", "[exporter]") {
  Environment env{1};
  EmptyReactor empty{"empty", env};
  env.assemble();
  auto buffer = export_reactor_graph_to_proto(env, std::nullopt);
  reactor_graph::Graph graph;
  graph.ParseFromString(buffer);

  SECTION("Exactly one empty reactor") {
    REQUIRE(graph.elements_size() == 1);
    const auto& elem = graph.elements(0);
    check_reactor_element(elem, empty);
  }
  SECTION("No connections") { REQUIRE(graph.connections_size() == 0); }
  SECTION("Exactly on empty containment") {
    REQUIRE(graph.containments_size() == 1);
    const auto& containment = graph.containments(0);
    REQUIRE(containment.container_uid() == empty.uid());
    REQUIRE(containment.containee_uids_size() == 0);
  }
  SECTION("No dependencies") { REQUIRE(graph.dependencies_size() == 0); }
}

TEST_CASE("Serialization of various reactor elements", "[exporter]") {
  Environment env{1};
  ElementsReactor elements{"elements", env};
  env.assemble();
  auto buffer = export_reactor_graph_to_proto(env, std::nullopt);
  reactor_graph::Graph graph;
  graph.ParseFromString(buffer);

  // There should be exactly 12 elements, 2 containments, and two empty dependencies
  REQUIRE(graph.elements_size() == 12);
  REQUIRE(graph.containments_size() == 2);
  REQUIRE(graph.connections_size() == 0);
  REQUIRE(graph.dependencies_size() == 2);

  SECTION("Check fields of the top-level reactor element") {
    const auto& elem_elements_reactor = lookup_element(graph, elements.uid());
    check_reactor_element(elem_elements_reactor, elements);
  }
  SECTION("Check fields of the contained empty reactor element") {
    const auto& elem_empty_reactor = lookup_element(graph, elements.empty.uid());
    check_reactor_element(elem_empty_reactor, elements.empty);
  }
  SECTION("Check fields of the startup element") {
    const auto& elem = lookup_element(graph, elements.startup.uid());
    check_startup_element(elem, elements.startup);
  }
  SECTION("Check fields of the shutdown element") {
    const auto& elem = lookup_element(graph, elements.shutdown.uid());
    check_shutdown_element(elem, elements.shutdown);
  }
  SECTION("Check fields of the timer1 element") {
    const auto& elem = lookup_element(graph, elements.timer1.uid());
    check_timer_element(elem, elements.timer1);
  }
  SECTION("Check fields of the timer2 element") {
    const auto& elem = lookup_element(graph, elements.timer2.uid());
    check_timer_element(elem, elements.timer2);
  }
  SECTION("Check fields of the laction element") {
    const auto& elem = lookup_element(graph, elements.laction.uid());
    check_action_element(elem, elements.laction, reactor_graph::ActionType::ACTION_TYPE_LOGICAL);
  }
  SECTION("Check fields of the paction element") {
    const auto& elem = lookup_element(graph, elements.paction.uid());
    check_action_element(elem, elements.paction, reactor_graph::ActionType::ACTION_TYPE_PHYSICAL);
  }
  SECTION("Check fields of the input element") {
    const auto& elem = lookup_element(graph, elements.input.uid());
    check_port_element(elem, elements.input, reactor_graph::PortType::PORT_TYPE_INPUT);
  }
  SECTION("Check fields of the output element") {
    const auto& elem = lookup_element(graph, elements.output.uid());
    check_port_element(elem, elements.output, reactor_graph::PortType::PORT_TYPE_OUTPUT);
  }
  SECTION("Check fields of the reaction element") {
    const auto& elem = lookup_element(graph, elements.reaction.uid());
    check_reaction_element(elem, elements.reaction);
  }
  SECTION("Check fields of the reaction2 element") {
    const auto& elem = lookup_element(graph, elements.reaction3.uid());
    check_reaction_element(elem, elements.reaction3);
  }
  SECTION("Check empty reaction dependencies") {
    const auto& dependency = graph.dependencies(0);
    REQUIRE(dependency.reaction_uid() == elements.reaction.uid());
    REQUIRE(dependency.trigger_uids_size() == 0);
    REQUIRE(dependency.effect_uids_size() == 0);
    REQUIRE(dependency.source_uids_size() == 0);
  }
  SECTION("Check empty containment") {
    const auto& containment = lookup_containment(graph, elements.empty.uid());
    REQUIRE(containment.containee_uids_size() == 0);
  }
  SECTION("Check elements reactor containment") {
    const auto& containment = lookup_containment(graph, elements.uid());
    REQUIRE(containment.containee_uids_size() == 11);
    std::set<std::uint64_t> uids;
    for (auto uid : containment.containee_uids()) {
      auto [_, success] = uids.insert(uid);
      REQUIRE(success);
    }
    for (auto* elem : elements.elements()) {
      REQUIRE(uids.count(elem->uid()) == 1);
    }
  }
}

void check_connection(const reactor_graph::Connection& connection, const BasePort& from,
                      std::initializer_list<const std::reference_wrapper<BasePort>> to) {
  REQUIRE(connection.from_uid() == from.uid());
  REQUIRE(std::size_t(connection.targets_size()) == to.size());
  std::set<std::uint64_t> uids;
  for (const auto& target : connection.targets()) {
    auto [_, success] = uids.insert(target.to_uid());
    REQUIRE(success);
  }
  for (const BasePort& port : to) {
    REQUIRE(uids.count(port.uid()) == 1);
  }
}

TEST_CASE("Test serialization of port connections", "[exporter]") {
  Environment env{1};
  SrcReactor src{"src", env};
  SrcReactor src_src{"src", src};
  SinkReactor sink1{"sink1", env};
  SinkReactor sink2{"sink2", env};
  SinkReactor sink3{"sink3", env};
  SinkReactor sink1_sink1{"sink1", sink1};
  SinkReactor sink1_sink2{"sink2", sink1};

  SECTION("Check regular connection") {
    env.draw_connection(src.output, sink1.input, {});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == src.output.uid());
    REQUIRE(connection.targets_size() == 1);
    const auto& target = connection.targets(0);
    REQUIRE(target.to_uid() == sink1.input.uid());
    REQUIRE(!target.properties().is_physical());
    REQUIRE(!target.properties().has_delay());
  }

  SECTION("Check physical connection") {
    env.draw_connection(src.output, sink1.input, {ConnectionType::Physical});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == src.output.uid());
    REQUIRE(connection.targets_size() == 1);
    const auto& target = connection.targets(0);
    REQUIRE(target.to_uid() == sink1.input.uid());
    REQUIRE(target.properties().is_physical());
    REQUIRE(!target.properties().has_delay());
  }

  SECTION("Check delayed connection") {
    env.draw_connection(src.output, sink1.input, {ConnectionType::Delayed, 2s});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == src.output.uid());
    REQUIRE(connection.targets_size() == 1);
    const auto& target = connection.targets(0);
    REQUIRE(target.to_uid() == sink1.input.uid());
    REQUIRE(!target.properties().is_physical());
    REQUIRE(target.properties().has_delay());
    REQUIRE(target.properties().delay().seconds() == 2);
    REQUIRE(target.properties().delay().nanos() == 0);
  }

  SECTION("Check zero delay connection") {
    env.draw_connection(src.output, sink1.input, {ConnectionType::Delayed});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == src.output.uid());
    REQUIRE(connection.targets_size() == 1);
    const auto& target = connection.targets(0);
    REQUIRE(target.to_uid() == sink1.input.uid());
    REQUIRE(!target.properties().is_physical());
    REQUIRE(target.properties().has_delay());
    REQUIRE(target.properties().delay().seconds() == 0);
    REQUIRE(target.properties().delay().nanos() == 0);
  }

  SECTION("Check fan-out connection") {
    env.draw_connection(src.output, sink1.input, {});
    env.draw_connection(src.output, sink2.input, {});
    env.draw_connection(src.output, sink3.input, {});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == src.output.uid());
    REQUIRE(connection.targets_size() == 3);
    std::set<std::uint64_t> uids = {sink1.input.uid(), sink2.input.uid(), sink3.input.uid()};
    std::set<std::uint64_t> target_uids;
    for (const auto& target : connection.targets()) {
      auto [_, success] = target_uids.insert(target.to_uid());
      REQUIRE(success);
      REQUIRE(!target.properties().is_physical());
      REQUIRE(!target.properties().has_delay());
    }
    REQUIRE(uids == target_uids);
  }

  SECTION("Check nested connection") {
    env.draw_connection(src_src.output, src.output, {});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == src_src.output.uid());
    REQUIRE(connection.targets_size() == 1);
    const auto& target = connection.targets(0);
    REQUIRE(target.to_uid() == src.output.uid());
    REQUIRE(!target.properties().is_physical());
    REQUIRE(!target.properties().has_delay());
  }

  SECTION("Check nested fan-out connection") {
    env.draw_connection(sink1.input, sink1_sink1.input, {});
    env.draw_connection(sink1.input, sink1_sink2.input, {});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 1);
    const auto& connection = graph.connections(0);
    REQUIRE(connection.from_uid() == sink1.input.uid());
    REQUIRE(connection.targets_size() == 2);
    std::set<std::uint64_t> uids = {sink1_sink1.input.uid(), sink1_sink2.input.uid()};
    std::set<std::uint64_t> target_uids;
    for (const auto& target : connection.targets()) {
      auto [_, success] = target_uids.insert(target.to_uid());
      REQUIRE(success);
      REQUIRE(!target.properties().is_physical());
      REQUIRE(!target.properties().has_delay());
    }
    REQUIRE(uids == target_uids);
  }

  SECTION("Check multiple connections") {
    env.draw_connection(src_src.output, src.output, {});
    env.draw_connection(src.output, sink1.input, {});
    env.draw_connection(src.output, sink2.input, {});
    env.draw_connection(src.output, sink3.input, {});
    env.draw_connection(sink1.input, sink1_sink1.input, {});
    env.draw_connection(sink1.input, sink1_sink2.input, {});
    env.assemble();
    reactor_graph::Graph graph;
    graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
    REQUIRE(graph.connections_size() == 3);
    std::set<std::uint64_t> uids = {src_src.output.uid(), src.output.uid(), sink1.input.uid()};
    std::set<std::uint64_t> from_uids;
    for (const auto& connection : graph.connections()) {
      auto [_, success] = from_uids.insert(connection.from_uid());
      REQUIRE(success);
    }
    REQUIRE(uids == from_uids);
  }
}

struct ReactionTest : public ElementsReactor {
  ReactionTest(const std::string& name, Environment& environment)
      : ElementsReactor(name, environment) {}

  ElementsReactor nested{"nested", *this};

  Reaction reaction2{"reaction2", 2, *this, []() {}};

  void assemble() override {
    reaction.declare_trigger(&startup);
    reaction.declare_trigger(&shutdown);
    reaction.declare_schedulable_action(&laction);
    reaction.declare_dependency(&input);
    reaction.declare_antidependency(&output);

    reaction2.declare_trigger(&paction);
    reaction2.declare_trigger(&nested.output);
    reaction2.declare_antidependency(&nested.input);
    reaction2.declare_antidependency(&output);
  }
};

TEST_CASE("Test serialization of reaction dependencies", "[exporter]") {
  Environment env{1};
  ReactionTest reactor("reactor", env);
  env.assemble();
  reactor_graph::Graph graph;
  graph.ParseFromString(export_reactor_graph_to_proto(env, std::nullopt));
  REQUIRE(graph.dependencies_size() == 5); // 2 ElementsReactors (one base class, one nested), each with 2 reactions,
                                           // plus one more reaction in the subclass

  SECTION("Check dependencies for reactor.reaction") {
    const auto& dependency = lookup_dependencies(graph, reactor.reaction.uid());
    std::set<std::uint64_t> trigger_uids;
    std::set<std::uint64_t> source_uids;
    std::set<std::uint64_t> effect_uids;
    for (auto uid : dependency.trigger_uids()) {
      auto [_, success] = trigger_uids.insert(uid);
      REQUIRE(success);
    }
    for (auto uid : dependency.source_uids()) {
      auto [_, success] = source_uids.insert(uid);
      REQUIRE(success);
    }
    for (auto uid : dependency.effect_uids()) {
      auto [_, success] = effect_uids.insert(uid);
      REQUIRE(success);
    }
    REQUIRE(trigger_uids == std::set<uint64_t>{reactor.startup.uid(), reactor.shutdown.uid()});
    REQUIRE(source_uids == std::set<uint64_t>{reactor.input.uid()});
    REQUIRE(effect_uids == std::set<uint64_t>{reactor.laction.uid(), reactor.output.uid()});
  }

  SECTION("Check dependencies for reactor.reaction2") {
    const auto& dependency = lookup_dependencies(graph, reactor.reaction2.uid());
    std::set<std::uint64_t> trigger_uids;
    std::set<std::uint64_t> effect_uids;
    for (auto uid : dependency.trigger_uids()) {
      auto [_, success] = trigger_uids.insert(uid);
      REQUIRE(success);
    }
    REQUIRE(dependency.source_uids_size() == 0);
    for (auto uid : dependency.effect_uids()) {
      auto [_, success] = effect_uids.insert(uid);
      REQUIRE(success);
    }
    REQUIRE(trigger_uids == std::set<uint64_t>{reactor.paction.uid(), reactor.nested.output.uid()});
    REQUIRE(effect_uids == std::set<uint64_t>{reactor.output.uid(), reactor.nested.input.uid()});
  }
}

// NOLINTNEXTLINE
class TestServer final : public diagram_generator::DiagramGenerator::Service {
  bool called{false};

  auto receive_graph([[maybe_unused]] grpc::ServerContext* context,
                     [[maybe_unused]] const diagram_generator::GraphWithMetadata* graph,
                     [[maybe_unused]] ::google::protobuf::Empty* empty) -> grpc::Status override {
    called = true;
    return grpc::Status::OK;
  }

public:
  TestServer() = default;
  ~TestServer() override { REQUIRE(called); }
};

TEST_CASE("Check exporter grpc call", "[exporter]") {
  Environment env{1};

  SECTION("Invalid host") {
    auto status = xronos::graph_exporter::detail::send_reactor_graph_to_diagram_server(env, std::nullopt, std::nullopt,
                                                                                       "invalid_host:424242");
    REQUIRE(!status.ok());
  }

  SECTION("Validate call to server") {
    std::string server_address("0.0.0.0:424242");
    TestServer service;
    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    auto server = builder.BuildAndStart();

    auto status = xronos::graph_exporter::detail::send_reactor_graph_to_diagram_server(env, std::nullopt, std::nullopt,
                                                                                       server_address);
    REQUIRE(status.ok());
  }
}
