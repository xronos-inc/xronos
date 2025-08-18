// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_GRAPH_EXPORTER_DETAIL_EXPORTER_HH
#define XRONOS_GRAPH_EXPORTER_DETAIL_EXPORTER_HH

#include <functional>
#include <optional>
#include <string>

#include "grpcpp/support/status.h"
#include "xronos/messages/source_info.pb.h"
#include "xronos/runtime/environment.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::graph_exporter::detail {

auto send_reactor_graph_to_diagram_server(
    const runtime::Environment& environment,
    const std::optional<xronos::messages::source_info::SourceInfo>& source_info,
    std::optional<std::reference_wrapper<const telemetry::AttributeManager>> attribute_manager, const std::string& host)
    -> ::grpc::Status;

} // namespace xronos::graph_exporter::detail

#endif // XRONOS_GRAPH_EXPORTER_DETAIL_EXPORTER_HH
