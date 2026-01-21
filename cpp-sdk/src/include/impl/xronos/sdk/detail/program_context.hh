// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IMPL_XRONOS_SDK_DETAIL_PROGRAM_CONTEXT_HH
#define IMPL_XRONOS_SDK_DETAIL_PROGRAM_CONTEXT_HH

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/telemetry.hh"

namespace xronos::sdk::detail {

struct ProgramContext {
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};
  telemetry::MetricDataLoggerProvider metric_data_logger_provider{};
  std::unique_ptr<telemetry::TelemetryBackend> telemetry_backend{std::make_unique<telemetry::NoopTelemetryBackend>()};
  source_location::SourceLocationRegistry source_location_registry{};
  std::unordered_map<std::uint64_t, std::function<void()>> assemble_callbacks{};
  std::unique_ptr<runtime::ProgramHandle> runtime_program_handle{nullptr};
};

} // namespace xronos::sdk::detail

#endif // IMPL_XRONOS_SDK_DETAIL_PROGRAM_CONTEXT_HH
