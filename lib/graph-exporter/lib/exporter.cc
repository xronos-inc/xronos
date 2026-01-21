// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/graph_exporter/exporter.hh"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include "google/protobuf/empty.pb.h"
#include "google/protobuf/util/time_util.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"
#include "grpcpp/support/status.h"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reaction_dependency_registry.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/messages/reactor_graph.pb.h"
#include "xronos/messages/source_info.pb.h"
#include "xronos/services/diagram_generator.grpc.pb.h"
#include "xronos/services/diagram_generator.pb.h"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::graph_exporter {

namespace detail {

class AttributeValueSetter {
public:
  AttributeValueSetter(messages::reactor_graph::Attribute& attribute)
      : attribute_{attribute} {}

  void operator()(const std::string& value) { attribute_.get().set_string(value); }
  void operator()(bool value) { attribute_.get().set_boolean(value); }
  void operator()(std::int64_t value) { attribute_.get().set_integer(value); }
  void operator()(double value) { attribute_.get().set_floatingpoint(value); }

private:
  std::reference_wrapper<messages::reactor_graph::Attribute> attribute_;
};

class ElementTypeSetter {
public:
  ElementTypeSetter(const telemetry::AttributeManager& attribute_manager, const core::ElementRegistry& element_registry,
                    messages::reactor_graph::ReactorElement& element)
      : attribute_manager_{attribute_manager}
      , element_registry_{element_registry}
      , element_{element} {}

  void operator()([[maybe_unused]] const core::InputPortTag& type) {
    auto& port = *element_.get().mutable_port();
    port.set_port_type(messages::reactor_graph::PortType::PORT_TYPE_INPUT);
  }

  void operator()(const core::MetricTag& type) {
    auto& metric = *element_.get().mutable_metric();
    metric.set_unit(type.properties->unit);
    metric.set_description(type.properties->description);

    auto attributes = attribute_manager_.get().get_attributes(element_.get().uid(), element_registry_);
    for (const auto& [key, value] : attributes) {
      auto& attribute = *metric.add_attributes();
      attribute.set_key(key);
      std::visit(AttributeValueSetter{attribute}, value);
    }
  }

  void operator()([[maybe_unused]] const core::OutputPortTag& type) {
    auto& port = *element_.get().mutable_port();
    port.set_port_type(messages::reactor_graph::PortType::PORT_TYPE_OUTPUT);
  }

  void operator()(const core::PeriodicTimerTag& type) {
    using TimeUtil = google::protobuf::util::TimeUtil;
    auto& timer = *element_.get().mutable_timer();
    *timer.mutable_offset() = TimeUtil::NanosecondsToDuration(type.properties->offset.count());
    *timer.mutable_period() = TimeUtil::NanosecondsToDuration(type.properties->period.count());
    timer.set_timer_type(messages::reactor_graph::TimerType::TIMER_TYPE_GENERIC);
  }

  void operator()([[maybe_unused]] const core::PhysicalEventTag& type) {
    auto& action = *element_.get().mutable_action();
    action.set_action_type(messages::reactor_graph::ActionType::ACTION_TYPE_PHYSICAL);
  }

  void operator()([[maybe_unused]] const core::ProgrammableTimerTag& type) {
    auto& action = *element_.get().mutable_action();
    action.set_action_type(messages::reactor_graph::ActionType::ACTION_TYPE_LOGICAL);
  }

  void operator()(const core::ReactionTag& type) {
    auto& reaction = *element_.get().mutable_reaction();
    reaction.set_priority(type.properties->position + 1);
  }

  void operator()([[maybe_unused]] const core::ReactorTag& type) { element_.get().mutable_reactor(); }

  void operator()([[maybe_unused]] const core::ShutdownTag& type) {
    auto& timer = *element_.get().mutable_timer();
    timer.set_timer_type(messages::reactor_graph::TimerType::TIMER_TYPE_SHUTDOWN);
  }

  void operator()([[maybe_unused]] const core::StartupTag& type) {
    auto& timer = *element_.get().mutable_timer();
    timer.set_timer_type(messages::reactor_graph::TimerType::TIMER_TYPE_STARTUP);
  }

private:
  std::reference_wrapper<const telemetry::AttributeManager> attribute_manager_;
  std::reference_wrapper<const core::ElementRegistry> element_registry_;
  std::reference_wrapper<messages::reactor_graph::ReactorElement> element_;
};

void serialize_reactor_elements(const core::ElementRegistry& element_registry,
                                const telemetry::AttributeManager& attribute_manager,
                                messages::reactor_graph::Graph& graph) {
  for (const auto& core_element : element_registry.elements()) {
    auto& proto_element = *graph.add_elements();
    proto_element.set_name(core_element.name);
    proto_element.set_uid(core_element.uid);
    std::visit(ElementTypeSetter{attribute_manager, element_registry, proto_element}, core_element.type);
  }
}

void serialize_connections(const core::ConnectionGraph& connection_graph, messages::reactor_graph::Graph& graph) {
  // The connection graph is indexed by target uid. We first convert it to a map
  // that is indexed by source uid.
  std::unordered_map<std::uint64_t, std::vector<const core::ConnectionProperties*>> connection_properties;
  for (const auto& properties : connection_graph.connections()) {
    connection_properties[properties.from_uid].push_back(&properties);
  }

  for (const auto& [from_uid, properties] : connection_properties) {
    auto& connection = *graph.add_connections();
    connection.set_from_uid(from_uid);
    for (const core::ConnectionProperties* props : properties) {
      auto& target = *connection.add_targets();
      target.set_to_uid(props->to_uid);
      auto& target_properties = *target.mutable_properties();
      target_properties.set_is_physical(false);
      if (props->delay.has_value()) {
        using TimeUtil = google::protobuf::util::TimeUtil;
        *(target_properties.mutable_delay()) = TimeUtil::NanosecondsToDuration(props->delay.value().count());
      }
    }
  }
}

auto serialize_containment(const core::ElementRegistry& element_registry, messages::reactor_graph::Graph& graph) {
  // First, create a map of container uids to their continee uids.
  std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> containees;
  // Add a map entry for each reactor
  for (const auto& reactor : element_registry.elements_of_type<core::ReactorTag>()) {
    auto res = containees.try_emplace(reactor.uid);
    util::assert_(res.second);
  };

  // Fill in the containees
  for (const auto& element : element_registry.elements()) {
    if (element.parent_uid.has_value()) {
      containees[element.parent_uid.value()].push_back(element.uid);
    }
  }

  // Serialize the data structure
  for (const auto& [container_uid, containee_uids] : containees) {
    auto& containment = *graph.add_containments();
    containment.set_container_uid(container_uid);
    for (auto containee_uid : containee_uids) {
      containment.add_containee_uids(containee_uid);
    }
  }
}

auto serialize_reaction_dependencies(const core::ElementRegistry& element_registry,
                                     const core::ReactionDependencyRegistry& reaction_dependency_registry,
                                     messages::reactor_graph::Graph& graph) {
  for (const auto& reaction : element_registry.elements_of_type<core::ReactionTag>()) {
    auto& dependencies = *graph.add_dependencies();
    dependencies.set_reaction_uid(reaction.uid);
    for (std::uint64_t trigger_uid : reaction_dependency_registry.get_triggers(reaction.uid)) {
      dependencies.add_trigger_uids(trigger_uid);
    }
    for (std::uint64_t effect_uid : reaction_dependency_registry.get_effects(reaction.uid)) {
      // The diagram server currently cannot visualize shutdown effects
      if (!std::holds_alternative<core::ShutdownTag>(element_registry.get(effect_uid).type)) {
        dependencies.add_effect_uids(effect_uid);
      }
    }
  }
}

void serialize_reactor_model(const core::ReactorModel& model, const telemetry::AttributeManager& attribute_manager,
                             messages::reactor_graph::Graph& graph) {
  serialize_reactor_elements(model.element_registry, attribute_manager, graph);
  serialize_connections(model.connection_graph, graph);
  serialize_containment(model.element_registry, graph);
  serialize_reaction_dependencies(model.element_registry, model.reaction_dependency_registry, graph);
}

void serialize_fqn(std::string_view fqn, messages::source_info::ElementSourceInfo& source_info) {
  std::size_t start = 0;
  std::size_t end = 0;
  while ((end = fqn.find('.', start)) != std::string_view::npos) {
    source_info.add_fqn(std::string(fqn.substr(start, end - start)));
    start = end + 1;
  }
  source_info.add_fqn(std::string(fqn.substr(start)));
}

void serialize_source_locations(const core::ElementRegistry& element_registry,
                                const source_location::SourceLocationRegistry& source_location_registry,
                                messages::source_info::SourceInfo& source_infos) {
  for (const auto& [uid, source_location] : source_location_registry.all_locations()) {
    auto* source_info = source_infos.add_infos();
    source_info->set_uid(uid);

    serialize_fqn(element_registry.get(uid).fqn, *source_info);

    auto* frame = source_info->mutable_frame();
    frame->set_file(source_location.file);
    frame->set_function(source_location.function);
    frame->set_lineno(source_location.start_line);
    frame->set_end_lineno(source_location.end_line);
    frame->set_col_offset(source_location.start_column);
    frame->set_end_col_offset(source_location.end_column);
  }
}

auto send_reactor_graph_to_diagram_server(const core::ReactorModel& model,
                                          const telemetry::AttributeManager& attribute_manager,
                                          const source_location::SourceLocationRegistry& source_location_registry,
                                          const std::string& host) -> ::grpc::Status {
  auto channel = ::grpc::CreateChannel(host, ::grpc::InsecureChannelCredentials());
  auto stub = services::diagram_generator::DiagramGenerator::NewStub(channel);
  ::grpc::ClientContext context;

  services::diagram_generator::GraphWithMetadata gwm{};

  serialize_source_locations(model.element_registry, source_location_registry, *gwm.mutable_source_info());

  serialize_reactor_model(model, attribute_manager, *gwm.mutable_graph());

  ::google::protobuf::Empty response;
  auto status = stub->receive_graph(&context, gwm, &response);

  return status;
}

auto get_diagram_server_endpoint() -> std::string {
  const char* diagram_server_endpoint_port = std::getenv("XRONOS_PROGRAM_INFO_PORT");
  if (diagram_server_endpoint_port == nullptr || strlen(diagram_server_endpoint_port) == 0) {
    diagram_server_endpoint_port = "50051";
  }
  const char* diagram_server_endpoint_host = std::getenv("XRONOS_PROGRAM_INFO_HOST");
  if (diagram_server_endpoint_host == nullptr || strlen(diagram_server_endpoint_host) == 0) {
    diagram_server_endpoint_host = "localhost";
  }
  return std::string{diagram_server_endpoint_host} + ":" + std::string{diagram_server_endpoint_port};
}

} // namespace detail

void send_reactor_graph_to_diagram_server(const core::ReactorModel& model,
                                          const telemetry::AttributeManager& attribute_manager,
                                          const source_location::SourceLocationRegistry& source_location_registry) {
  std::string host{detail::get_diagram_server_endpoint()};
  util::log::debug() << "Sending reactor graph to diagram server endpoint at " << host;

  auto status = detail::send_reactor_graph_to_diagram_server(model, attribute_manager, source_location_registry, host);

  constexpr int ERROR_CODE_FAILED_TO_CONNECT = 14;
  if (!status.ok() && status.error_code() != ERROR_CODE_FAILED_TO_CONNECT) {
    util::log::warn() << "Sending the reactor graph for rendering failed with message: " << status.error_message();
    return;
  }
}

} // namespace xronos::graph_exporter
