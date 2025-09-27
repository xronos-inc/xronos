// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <optional>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string>
#include <variant>

#include "catch2/catch_test_macros.hpp"
#include "google/protobuf/empty.pb.h"
#include "google/protobuf/util/time_util.h"
#include "grpcpp/completion_queue.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/graph_exporter/exporter.hh"
#include "xronos/messages/reactor_graph.pb.h"
#include "xronos/services/diagram_generator.grpc.pb.h"
#include "xronos/services/diagram_generator.pb.h"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/util/assert.hh"

using namespace std::literals::chrono_literals;
using namespace xronos::messages;
using namespace xronos::services;
using namespace xronos::graph_exporter;
using namespace xronos;

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

auto lookup_dependencies(const reactor_graph::Graph& graph, std::uint64_t uid)
    -> const reactor_graph::ReactionDependencies& {
  for (const auto& elem : graph.dependencies()) {
    if (elem.reaction_uid() == uid) {
      return elem;
    }
  }
  throw std::runtime_error("dependencies not found");
}

void check_base_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem) {
  REQUIRE(graph_elem.uid() == core_elem.uid);
  REQUIRE(graph_elem.name() == core_elem.name);
}

void check_reactor_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem) {
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_reactor());
}

void check_periodic_timer_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_timer());
  const auto& elem_timer = graph_elem.timer();
  util::assert_(std::holds_alternative<core::PeriodicTimerTag>(core_elem.type));
  const auto& properties = *std::get<core::PeriodicTimerTag>(core_elem.type).properties;
  REQUIRE(elem_timer.timer_type() == reactor_graph::TimerType::TIMER_TYPE_GENERIC);
  REQUIRE(TimeUtil::DurationToNanoseconds(elem_timer.offset()) == properties.offset.count());
  REQUIRE(TimeUtil::DurationToNanoseconds(elem_timer.period()) == properties.period.count());
}

void check_startup_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_timer());
  const auto& elem_timer = graph_elem.timer();
  REQUIRE(elem_timer.timer_type() == reactor_graph::TimerType::TIMER_TYPE_STARTUP);
}

void check_shutdown_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_timer());
  const auto& elem_timer = graph_elem.timer();
  REQUIRE(elem_timer.timer_type() == reactor_graph::TimerType::TIMER_TYPE_SHUTDOWN);
}

void check_action_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem,
                          reactor_graph::ActionType expected_action_type) {
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_action());
  const auto& elem_action = graph_elem.action();
  REQUIRE(elem_action.action_type() == expected_action_type);
}

void check_port_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem,
                        reactor_graph::PortType expected_port_type) {
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_port());
  const auto& elem_port = graph_elem.port();
  REQUIRE(elem_port.port_type() == expected_port_type);
}

void check_reaction_element(const reactor_graph::ReactorElement& graph_elem, const core::Element& core_elem) {
  check_base_element(graph_elem, core_elem);
  REQUIRE(graph_elem.has_reaction());
  const auto& elem_reaction = graph_elem.reaction();
  util::assert_(std::holds_alternative<core::ReactionTag>(core_elem.type));
  const auto& properties = *std::get<core::ReactionTag>(core_elem.type).properties;
  REQUIRE(elem_reaction.priority() == properties.position + 1);
  REQUIRE_FALSE(elem_reaction.has_deadline());
}

TEST_CASE("Serialization of a single empty reactor", "[exporter]") {
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};
  const auto& empty = model.element_registry.add_new_element("empty", core::ReactorTag{}, std::nullopt);

  reactor_graph::Graph graph{};
  graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);

  SECTION("Exactly one empty reactor") {
    REQUIRE(graph.elements_size() == 1);
    const auto& elem = graph.elements(0);
    check_reactor_element(elem, empty);
  }
  SECTION("No connections") { REQUIRE(graph.connections_size() == 0); }
  SECTION("Exactly one empty containment") {
    REQUIRE(graph.containments_size() == 1);
    const auto& containment = graph.containments(0);
    REQUIRE(containment.container_uid() == empty.uid);
    REQUIRE(containment.containee_uids_size() == 0);
  }
  SECTION("No dependencies") { REQUIRE(graph.dependencies_size() == 0); }
}

TEST_CASE("Serialization of various reactor elements", "[exporter]") {
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};

  const auto& elements = model.element_registry.add_new_element("elements", core::ReactorTag{}, std::nullopt);
  const auto& startup = model.element_registry.add_new_element("startup", core::StartupTag{}, elements.uid);
  const auto& shutdown = model.element_registry.add_new_element("shutdown", core::ShutdownTag{}, elements.uid);
  const auto& timer1 = model.element_registry.add_new_element(
      "timer1", core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>(1s, 2s)}, elements.uid);
  const auto& timer2 = model.element_registry.add_new_element(
      "timer2", core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>(12ms, 42us)}, elements.uid);
  const auto& programmable =
      model.element_registry.add_new_element("programmable", core::ProgrammableTimerTag{}, elements.uid);
  const auto& physical_event =
      model.element_registry.add_new_element("physical_event", core::PhysicalEventTag{}, elements.uid);
  const auto& input = model.element_registry.add_new_element("input", core::InputPortTag{}, elements.uid);
  const auto& output = model.element_registry.add_new_element("output", core::OutputPortTag{}, elements.uid);
  const auto& reaction1 = model.element_registry.add_new_element(
      "reaction1", core::ReactionTag{std::make_unique<core::ReactionProperties>([]() {}, 0)}, elements.uid);
  const auto& reaction3 = model.element_registry.add_new_element(
      "reaction3", core::ReactionTag{std::make_unique<core::ReactionProperties>([]() {}, 2)}, elements.uid);
  const auto& empty = model.element_registry.add_new_element("empty", core::ReactorTag{}, elements.uid);

  reactor_graph::Graph graph{};
  graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);

  // There should be exactly 12 elements, 2 containments, and two empty dependencies
  REQUIRE(graph.elements_size() == 12);
  REQUIRE(graph.containments_size() == 2);
  REQUIRE(graph.connections_size() == 0);
  REQUIRE(graph.dependencies_size() == 2);

  SECTION("Check fields of the top-level reactor element") {
    const auto& graph_elem = lookup_element(graph, elements.uid);
    check_reactor_element(graph_elem, elements);
  }
  SECTION("Check fields of the contained empty reactor element") {
    const auto& graph_elem = lookup_element(graph, empty.uid);
    check_reactor_element(graph_elem, empty);
  }
  SECTION("Check fields of the startup element") {
    const auto& graph_elem = lookup_element(graph, startup.uid);
    check_startup_element(graph_elem, startup);
  }
  SECTION("Check fields of the shutdown element") {
    const auto& graph_elem = lookup_element(graph, shutdown.uid);
    check_shutdown_element(graph_elem, shutdown);
  }
  SECTION("Check fields of the timer1 element") {
    const auto& graph_elem = lookup_element(graph, timer1.uid);
    check_periodic_timer_element(graph_elem, timer1);
  }
  SECTION("Check fields of the timer2 element") {
    const auto& graph_elem = lookup_element(graph, timer2.uid);
    check_periodic_timer_element(graph_elem, timer2);
  }
  SECTION("Check fields of the programmable timer element") {
    const auto& graph_elem = lookup_element(graph, programmable.uid);
    check_action_element(graph_elem, programmable, reactor_graph::ActionType::ACTION_TYPE_LOGICAL);
  }
  SECTION("Check fields of the physical event element") {
    const auto& graph_elem = lookup_element(graph, physical_event.uid);
    check_action_element(graph_elem, physical_event, reactor_graph::ActionType::ACTION_TYPE_PHYSICAL);
  }
  SECTION("Check fields of the input element") {
    const auto& graph_elem = lookup_element(graph, input.uid);
    check_port_element(graph_elem, input, reactor_graph::PortType::PORT_TYPE_INPUT);
  }
  SECTION("Check fields of the output element") {
    const auto& graph_elem = lookup_element(graph, output.uid);
    check_port_element(graph_elem, output, reactor_graph::PortType::PORT_TYPE_OUTPUT);
  }
  SECTION("Check fields of the reaction1 element") {
    const auto& graph_elem = lookup_element(graph, reaction1.uid);
    check_reaction_element(graph_elem, reaction1);
  }
  SECTION("Check fields of the reaction3 element") {
    const auto& graph_elem = lookup_element(graph, reaction3.uid);
    check_reaction_element(graph_elem, reaction3);
  }
  SECTION("Check empty reaction dependencies") {
    const auto& dependency = graph.dependencies(0);
    REQUIRE(dependency.reaction_uid() == reaction1.uid);
    REQUIRE(dependency.trigger_uids_size() == 0);
    REQUIRE(dependency.effect_uids_size() == 0);
    REQUIRE(dependency.source_uids_size() == 0);
  }
  SECTION("Check empty containment") {
    const auto& containment = lookup_containment(graph, empty.uid);
    REQUIRE(containment.containee_uids_size() == 0);
  }
  SECTION("Check elements reactor containment") {
    const auto& containment = lookup_containment(graph, elements.uid);
    REQUIRE(containment.containee_uids_size() == 11);
    std::set<std::uint64_t> uids{};
    for (auto uid : containment.containee_uids()) {
      auto [_, success] = uids.insert(uid);
      REQUIRE(success);
    }
    for (const auto& core_elem : model.element_registry.elements() | std::views::filter([](const auto& elem) {
                                   return elem.name != "elements" && elem.name != "empty";
                                 })) {
      REQUIRE(uids.contains(core_elem.uid));
    }
  }
}

void check_connection(const reactor_graph::Connection& connection, std::uint64_t from_uid,
                      std::initializer_list<std::uint64_t> to_uids,
                      std::optional<core::Duration> delay = std::nullopt) {
  REQUIRE(connection.from_uid() == from_uid);
  REQUIRE(std::size_t(connection.targets_size()) == to_uids.size());
  std::set<std::uint64_t> uids;
  for (const auto& target : connection.targets()) {
    auto [_, success] = uids.insert(target.to_uid());
    REQUIRE(success);
    REQUIRE_FALSE(target.properties().is_physical());
    if (delay.has_value()) {
      REQUIRE(target.properties().has_delay());
      using TimeUtil = google::protobuf::util::TimeUtil;
      REQUIRE(TimeUtil::DurationToNanoseconds(target.properties().delay()) == delay.value().count());
    } else {
      REQUIRE_FALSE(target.properties().has_delay());
    }
  }
  for (std::uint64_t to_uid : to_uids) {
    REQUIRE(uids.contains(to_uid));
  }
}

TEST_CASE("Test serialization of port connections", "[exporter]") {
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};

  const auto& src = model.element_registry.add_new_element("src", core::ReactorTag{}, std::nullopt);
  const auto& src_output = model.element_registry.add_new_element("output", core::OutputPortTag{}, src.uid);
  const auto& src_src = model.element_registry.add_new_element("src", core::ReactorTag{}, src.uid);
  const auto& src_src_output = model.element_registry.add_new_element("output", core::OutputPortTag{}, src_src.uid);
  const auto& sink1 = model.element_registry.add_new_element("sink1", core::ReactorTag{}, std::nullopt);
  const auto& sink1_input = model.element_registry.add_new_element("input", core::InputPortTag{}, sink1.uid);
  const auto& sink2 = model.element_registry.add_new_element("sink2", core::ReactorTag{}, std::nullopt);
  const auto& sink2_input = model.element_registry.add_new_element("input", core::InputPortTag{}, sink2.uid);
  const auto& sink3 = model.element_registry.add_new_element("sink3", core::ReactorTag{}, std::nullopt);
  const auto& sink3_input = model.element_registry.add_new_element("input", core::InputPortTag{}, sink3.uid);
  const auto& sink1_sink1 = model.element_registry.add_new_element("sink1", core::ReactorTag{}, std::nullopt);
  const auto& sink1_sink1_input =
      model.element_registry.add_new_element("input", core::InputPortTag{}, sink1_sink1.uid);
  const auto& sink1_sink2 = model.element_registry.add_new_element("sink2", core::ReactorTag{}, std::nullopt);
  const auto& sink1_sink2_input =
      model.element_registry.add_new_element("input", core::InputPortTag{}, sink1_sink2.uid);

  SECTION("Check regular connection") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink1_input.uid});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 1);
    check_connection(graph.connections(0), src_output.uid, {sink1_input.uid});
  }

  SECTION("Check delayed connection") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink1_input.uid, .delay = 2s});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 1);
    check_connection(graph.connections(0), src_output.uid, {sink1_input.uid}, 2s);
  }

  SECTION("Check zero delay connection") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink1_input.uid, .delay = 0s});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 1);
    check_connection(graph.connections(0), src_output.uid, {sink1_input.uid}, 0s);
  }

  SECTION("Check fan-out connection") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink1_input.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink2_input.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink3_input.uid});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 1);
    check_connection(graph.connections(0), src_output.uid, {sink1_input.uid, sink2_input.uid, sink3_input.uid});
  }

  SECTION("Check nested connection") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_src_output.uid, .to_uid = src_output.uid});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 1);
    check_connection(graph.connections(0), src_src_output.uid, {src_output.uid});
  }

  SECTION("Check nested fan-out connection") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = sink1_input.uid, .to_uid = sink1_sink1_input.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = sink1_input.uid, .to_uid = sink1_sink2_input.uid});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 1);
    check_connection(graph.connections(0), sink1_input.uid, {sink1_sink1_input.uid, sink1_sink2_input.uid});
  }

  SECTION("Check multiple connections") {
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_src_output.uid, .to_uid = src_output.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink1_input.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink2_input.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = src_output.uid, .to_uid = sink3_input.uid});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = sink1_input.uid, .to_uid = sink1_sink1_input.uid, .delay = 1s});
    model.connection_graph.add_connection(
        core::ConnectionProperties{.from_uid = sink1_input.uid, .to_uid = sink1_sink2_input.uid, .delay = 1s});
    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);
    REQUIRE(graph.connections_size() == 3);
    for (const auto& connection : graph.connections()) {
      std::uint64_t from_uid = connection.from_uid();
      if (from_uid == src_src_output.uid) {
        check_connection(connection, from_uid, {src_output.uid});
      } else if (from_uid == src_output.uid) {
        check_connection(connection, from_uid, {sink1_input.uid, sink2_input.uid, sink3_input.uid});
      } else if (from_uid == sink1_input.uid) {
        check_connection(connection, from_uid, {sink1_sink1_input.uid, sink1_sink2_input.uid}, 1s);
      } else {
        REQUIRE(false); // should never be reached
      }
    }
  }
}

TEST_CASE("Test serialization of reaction dependencies", "[exporter]") {
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};

  const auto& reaction1 = model.element_registry.add_new_element(
      "reaction1", core::ReactionTag{std::make_unique<core::ReactionProperties>(nullptr, 0)}, std::nullopt);
  const auto& reaction2 = model.element_registry.add_new_element(
      "reaction2", core::ReactionTag{std::make_unique<core::ReactionProperties>(nullptr, 1)}, std::nullopt);
  const auto& elem1 = model.element_registry.add_new_element("elem1", core::InputPortTag{}, std::nullopt);
  const auto& elem2 = model.element_registry.add_new_element("elem2", core::InputPortTag{}, std::nullopt);
  const auto& elem3 = model.element_registry.add_new_element("elem3", core::InputPortTag{}, std::nullopt);
  const auto& elem4 = model.element_registry.add_new_element("elem4", core::InputPortTag{}, std::nullopt);

  SECTION("Check dependencies for reaction1") {
    model.reaction_dependency_registry.register_reaction_effect(reaction1.uid, elem1.uid);
    model.reaction_dependency_registry.register_reaction_effect(reaction1.uid, elem2.uid);
    model.reaction_dependency_registry.register_reaction_effect(reaction1.uid, elem3.uid);
    model.reaction_dependency_registry.register_reaction_trigger(reaction1.uid, elem4.uid);

    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);

    const auto& dependency = lookup_dependencies(graph, reaction1.uid);
    std::set<std::uint64_t> trigger_uids;
    std::set<std::uint64_t> effect_uids;
    for (auto uid : dependency.trigger_uids()) {
      auto [_, success] = trigger_uids.insert(uid);
      REQUIRE(success);
    }
    for (auto uid : dependency.effect_uids()) {
      auto [_, success] = effect_uids.insert(uid);
      REQUIRE(success);
    }
    REQUIRE(trigger_uids == std::set<uint64_t>{elem4.uid});
    REQUIRE(effect_uids == std::set<uint64_t>{elem1.uid, elem2.uid, elem3.uid});
    REQUIRE(dependency.source_uids().empty());

    const auto& reaction2_dependency = lookup_dependencies(graph, reaction2.uid);
    REQUIRE(reaction2_dependency.trigger_uids().empty());
    REQUIRE(reaction2_dependency.source_uids().empty());
    REQUIRE(reaction2_dependency.effect_uids().empty());
  }

  SECTION("Check dependencies for reaction2") {
    model.reaction_dependency_registry.register_reaction_trigger(reaction2.uid, elem1.uid);
    model.reaction_dependency_registry.register_reaction_trigger(reaction2.uid, elem2.uid);
    model.reaction_dependency_registry.register_reaction_trigger(reaction2.uid, elem3.uid);
    model.reaction_dependency_registry.register_reaction_effect(reaction2.uid, elem4.uid);

    reactor_graph::Graph graph{};
    graph_exporter::detail::serialize_reactor_model(model, attribute_manager, graph);

    const auto& dependency = lookup_dependencies(graph, reaction2.uid);
    std::set<std::uint64_t> trigger_uids;
    std::set<std::uint64_t> effect_uids;
    for (auto uid : dependency.trigger_uids()) {
      auto [_, success] = trigger_uids.insert(uid);
      REQUIRE(success);
    }
    for (auto uid : dependency.effect_uids()) {
      auto [_, success] = effect_uids.insert(uid);
      REQUIRE(success);
    }
    REQUIRE(trigger_uids == std::set<uint64_t>{elem1.uid, elem2.uid, elem3.uid});
    REQUIRE(effect_uids == std::set<uint64_t>{elem4.uid});
    REQUIRE(dependency.source_uids().empty());

    const auto& reaction1_dependency = lookup_dependencies(graph, reaction1.uid);
    REQUIRE(reaction1_dependency.trigger_uids().empty());
    REQUIRE(reaction1_dependency.source_uids().empty());
    REQUIRE(reaction1_dependency.effect_uids().empty());
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
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};
  source_location::SourceLocationRegistry source_location_registry{};

  SECTION("Invalid host") {
    auto status = xronos::graph_exporter::detail::send_reactor_graph_to_diagram_server(
        model, attribute_manager, source_location_registry, "invalid_host:424242");
    REQUIRE(!status.ok());
  }

  SECTION("Validate call to server") {
    std::string server_address("0.0.0.0:424242");
    TestServer service;
    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    auto server = builder.BuildAndStart();

    auto status = xronos::graph_exporter::detail::send_reactor_graph_to_diagram_server(
        model, attribute_manager, source_location_registry, server_address);
    REQUIRE(status.ok());
  }
}
