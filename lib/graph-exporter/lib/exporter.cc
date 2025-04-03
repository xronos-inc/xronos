// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cassert>
#include <stdexcept>
#include <string>
#include <utility>

#include "xronos/graph_exporter/detail/exporter.hh"
#include "xronos/graph_exporter/exporter.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/assert.hh"
#include "xronos/runtime/misc_element.hh"
#include "xronos/runtime/port.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/runtime/reactor.hh"

#include "xronos/messages/reactor_graph.pb.h"
#include "xronos/messages/source_info.pb.h"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/services/diagram_generator.grpc.pb.h"

#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/time_util.h>

#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

using namespace xronos::messages;
using namespace xronos::services;
using namespace xronos::runtime;

void export_containment(const Reactor& reactor, reactor_graph::Graph& graph) {
  // Must be kept in sync with the GraphExporterVisitor to export a UID if and
  // only if the element is exported to the graph's element list.
  class MaybeExportUid : public ReactorElementVisitor {
  private:
    std::function<void(const ReactorElement&)> export_uid;

  public:
    MaybeExportUid(std::function<void(const ReactorElement&)> export_uid)
        : export_uid{std::move(export_uid)} {}

    void visit(const Reactor& element) final { export_uid(element); }
    void visit(const Reaction& element) final { export_uid(element); }
    void visit(const BaseAction& element) final { export_uid(element); }
    void visit(const BasePort& element) final { export_uid(element); }
    void visit(const Timer& element) final { export_uid(element); }
    void visit(const StartupTrigger& element) final { export_uid(element); }
    void visit(const ShutdownTrigger& element) final { export_uid(element); }
    void visit([[maybe_unused]] const MiscElement& misc) final {} // do not export misc elements
  };
  auto& containment = *graph.add_containments();
  containment.set_container_uid(reactor.uid());
  for (const auto* element : reactor.elements()) {
    MaybeExportUid visitor{
        [&containment](const ReactorElement& element) { containment.add_containee_uids(element.uid()); }};
    element->visit(visitor);
  }
}

void export_reaction_dependencies(const Reaction& reaction, reactor_graph::Graph& graph) {
  auto& dependencies = *graph.add_dependencies();
  dependencies.set_reaction_uid(reaction.uid());
  for (auto* trigger : reaction.action_triggers()) {
    dependencies.add_trigger_uids(trigger->uid());
  }
  for (auto* trigger : reaction.port_triggers()) {
    dependencies.add_trigger_uids(trigger->uid());
  }
  for (auto* source : reaction.dependencies()) {
    // dependencies also include the port triggers.
    // -> all dependencies that are not triggers are sources.
    if (!reaction.port_triggers().contains(source)) {
      dependencies.add_source_uids(source->uid());
    }
  }
  for (auto* source : reaction.action_dependencies()) {
    dependencies.add_source_uids(source->uid());
  }
  for (auto* effect : reaction.antidependencies()) {
    dependencies.add_effect_uids(effect->uid());
  }
  for (auto* effect : reaction.scheduable_actions()) {
    dependencies.add_effect_uids(effect->uid());
  }
}

auto add_new_element(const ReactorElement& element, reactor_graph::Graph& graph) -> reactor_graph::ReactorElement& {
  auto& graph_elem = *graph.add_elements();
  graph_elem.set_name(element.name());
  graph_elem.set_uid(element.uid());
  return graph_elem;
}

void export_reaction(const Reaction& reaction, reactor_graph::Graph& graph) {
  auto& elem = add_new_element(reaction, graph);
  auto& elem_reaction = *elem.mutable_reaction();
  elem_reaction.set_priority(reaction.priority());
  if (reaction.has_deadline()) {
    using TimeUtil = google::protobuf::util::TimeUtil;
    *elem_reaction.mutable_deadline() = TimeUtil::NanosecondsToDuration(reaction.deadline().count());
  }

  export_reaction_dependencies(reaction, graph);
}

void export_port(const BasePort& port, reactor_graph::Graph& graph) {
  auto& elem = add_new_element(port, graph);
  auto& port_elem = *elem.mutable_port();
  reactor_assert(port.is_input() || port.is_output());
  if (port.is_input()) {
    port_elem.set_port_type(reactor_graph::PortType::PORT_TYPE_INPUT);
  } else {
    port_elem.set_port_type(reactor_graph::PortType::PORT_TYPE_OUTPUT);
  }
  // FIXME There is currently no way to get the port data type
}

void export_timer(const Timer& timer, reactor_graph::Graph& graph) {
  using TimeUtil = google::protobuf::util::TimeUtil;
  auto& elem = add_new_element(timer, graph);
  auto& elem_timer = *elem.mutable_timer();
  *elem_timer.mutable_offset() = TimeUtil::NanosecondsToDuration(timer.offset().count());
  *elem_timer.mutable_period() = TimeUtil::NanosecondsToDuration(timer.period().count());
  elem_timer.set_timer_type(reactor_graph::TimerType::TIMER_TYPE_GENERIC);
}

void export_startup(const StartupTrigger& startup, reactor_graph::Graph& graph) {
  auto& elem = add_new_element(startup, graph);
  auto& elem_timer = *elem.mutable_timer();
  elem_timer.set_timer_type(reactor_graph::TimerType::TIMER_TYPE_STARTUP);
}

void export_shutdown(const ShutdownTrigger& shutdown, reactor_graph::Graph& graph) {
  auto& elem = add_new_element(shutdown, graph);
  auto& elem_timer = *elem.mutable_timer();
  elem_timer.set_timer_type(reactor_graph::TimerType::TIMER_TYPE_SHUTDOWN);
}

void export_action(const BaseAction& action, reactor_graph::Graph& graph) {
  auto& elem = add_new_element(action, graph);
  auto& elem_action = *elem.mutable_action();
  reactor_assert(action.is_logical() || action.is_physical());
  if (action.is_logical()) {
    elem_action.set_action_type(reactor_graph::ActionType::ACTION_TYPE_LOGICAL);
  }
  if (action.is_physical()) {
    elem_action.set_action_type(reactor_graph::ActionType::ACTION_TYPE_PHYSICAL);
  }
  // FIXME There is currently no way to get the action data type
}

void export_reactor(const Reactor& reactor, reactor_graph::Graph& graph) {
  auto& elem = add_new_element(reactor, graph);
  elem.mutable_reactor();

  // export the containment relation for this reactor
  export_containment(reactor, graph);
}

void export_connections(const Environment& environment, reactor_graph::Graph& graph) {
  for (const auto& [from, edges] : environment.connections()) {
    auto& connection = *graph.add_connections();
    connection.set_from_uid(from->uid());
    for (const auto& [properties, to] : edges) {
      auto& target = *connection.add_targets();
      target.set_to_uid(to->uid());
      auto& target_properties = *target.mutable_properties();
      if (properties.type_ == ConnectionType::Physical) {
        target_properties.set_is_physical(true);
      } else if (properties.type_ == ConnectionType::Delayed) {
        using TimeUtil = google::protobuf::util::TimeUtil;
        *(target_properties.mutable_delay()) = TimeUtil::NanosecondsToDuration(properties.delay_.count());
      }
    }
  }
}

class GraphExporterVisitor : public ReactorElementVisitor {
private:
  std::reference_wrapper<reactor_graph::Graph> graph;

public:
  GraphExporterVisitor(reactor_graph::Graph& graph)
      : graph{graph} {}

  void visit(const Reactor& reactor) final { export_reactor(reactor, graph); }
  void visit(const Reaction& reaction) final { export_reaction(reaction, graph); }
  void visit(const BaseAction& action) final { export_action(action, graph); }
  void visit(const BasePort& port) final { export_port(port, graph); }
  void visit(const Timer& timer) final { export_timer(timer, graph); }
  void visit(const StartupTrigger& startup) final { export_startup(startup, graph); }
  void visit(const ShutdownTrigger& shutdown) final { export_shutdown(shutdown, graph); }
  void visit([[maybe_unused]] const MiscElement& misc) final {}
};

auto export_reactor_graph(const Environment& environment, reactor_graph::Graph& graph) {
  GraphExporterVisitor visitor{graph};
  environment.visit_all_elements(visitor);
  export_connections(environment, graph);
}

namespace xronos::graph_exporter {

auto export_reactor_graph_to_proto(const Environment& environment) -> std::string {
  reactor_graph::Graph graph;
  export_reactor_graph(environment, graph);

  std::string buffer;
  graph.SerializeToString(&buffer);
  return buffer;
}

auto export_reactor_graph_to_json(const Environment& environment,
                                  const std::optional<source_info::SourceInfo>& source_info,
                                  bool pretty) -> std::string {
  using namespace google::protobuf::util;

  diagram_generator::GraphWithMetadata gwm;
  if (source_info.has_value()) {
    *gwm.mutable_source_info() = source_info.value();
  }

  reactor_graph::Graph graph;
  export_reactor_graph(environment, graph);

  *gwm.mutable_graph() = graph;

  std::string buffer;
  JsonPrintOptions options;
  options.add_whitespace = pretty;
  auto status = MessageToJsonString(gwm, &buffer, options);
  if (!status.ok()) {
    throw std::runtime_error(std::string{status.message()});
  }
  return buffer;
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

namespace detail {

auto send_reactor_graph_to_diagram_server(const Environment& environment,
                                          const std::optional<source_info::SourceInfo>& source_info,
                                          const std::string& host) -> ::grpc::Status {

  auto channel = grpc::CreateChannel(host, grpc::InsecureChannelCredentials());
  auto stub = diagram_generator::DiagramGenerator::NewStub(channel);
  grpc::ClientContext context;

  diagram_generator::GraphWithMetadata gwm;
  if (source_info.has_value()) {
    *gwm.mutable_source_info() = source_info.value();
  }

  reactor_graph::Graph graph;
  export_reactor_graph(environment, graph);

  *gwm.mutable_graph() = graph;

  ::google::protobuf::Empty response;
  auto status = stub->receive_graph(&context, gwm, &response);

  return status;
}

} // namespace detail

void send_reactor_graph_to_diagram_server(const Environment& environment,
                                          const std::optional<source_info::SourceInfo>& source_info) {
  std::string host{get_diagram_server_endpoint()};
  log::Debug() << "Sending reactor graph to diagram server endpoint at " << host;

  auto status = detail::send_reactor_graph_to_diagram_server(environment, source_info, host);

  constexpr int ERROR_CODE_FAILED_TO_CONNECT = 14;
  if (!status.ok() && status.error_code() != ERROR_CODE_FAILED_TO_CONNECT) {
    log::Warn() << "Sending the reactor graph for rendering failed with message: " << status.error_message();
    return;
  }
}

} // namespace xronos::graph_exporter
