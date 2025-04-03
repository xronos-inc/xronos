// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_GRAPH_EXPORTER_EXPORTER_HH
#define XRONOS_GRAPH_EXPORTER_EXPORTER_HH

#include "xronos/runtime/environment.hh"

#include "xronos/messages/source_info.pb.h"

#include <grpcpp/support/status.h>

namespace xronos::graph_exporter {

auto export_reactor_graph_to_proto(const runtime::Environment& environment) -> std::string;

auto export_reactor_graph_to_json(const runtime::Environment& environment,
                                  const std::optional<xronos::messages::source_info::SourceInfo>& source_info,
                                  bool pretty) -> std::string;

void send_reactor_graph_to_diagram_server(const runtime::Environment& environment,
                                          const std::optional<xronos::messages::source_info::SourceInfo>& source_info);

} // namespace xronos::graph_exporter

#endif // XRONOS_GRAPH_EXPORTER_EXPORTER_HH
