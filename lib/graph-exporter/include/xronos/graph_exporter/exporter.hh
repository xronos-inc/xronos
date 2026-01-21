// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_GRAPH_EXPORTER_EXPORTER_HH
#define XRONOS_GRAPH_EXPORTER_EXPORTER_HH

#include <string>

#include "grpcpp/support/status.h"
#include "xronos/core/reactor_model.hh"
#include "xronos/messages/reactor_graph.pb.h"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::graph_exporter {

void send_reactor_graph_to_diagram_server(const core::ReactorModel& model,
                                          const telemetry::AttributeManager& attribute_manager,
                                          const source_location::SourceLocationRegistry& source_location_registry);

namespace detail {

void serialize_reactor_model(const core::ReactorModel& model, const telemetry::AttributeManager& attribute_manager,
                             messages::reactor_graph::Graph& graph);

auto send_reactor_graph_to_diagram_server(const core::ReactorModel& model,
                                          const telemetry::AttributeManager& attribute_manager,
                                          const source_location::SourceLocationRegistry& source_location_registry,
                                          const std::string& host) -> ::grpc::Status;

} // namespace detail

} // namespace xronos::graph_exporter

#endif // XRONOS_GRAPH_EXPORTER_EXPORTER_HH
