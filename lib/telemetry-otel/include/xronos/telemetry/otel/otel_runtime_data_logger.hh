// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_OTEL_OTEL_RUNTIME_DATA_LOGGER_HH
#define XRONOS_TELEMETRY_OTEL_OTEL_RUNTIME_DATA_LOGGER_HH

#include "xronos/runtime/data_logger.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::telemetry::otel {

class OtelRuntimeDataLogger : public runtime::RuntimeDataLogger {
private:
  std::reference_wrapper<xronos::telemetry::AttributeManager> attribute_manager_;

public:
  OtelRuntimeDataLogger(xronos::telemetry::AttributeManager& attribute_manager)
      : attribute_manager_(attribute_manager) {}
  auto record_reaction_start(const runtime::Reaction& reaction) -> runtime::RuntimeDataLogger::ReactionScopePtr final;
};

} // namespace xronos::telemetry::otel

#endif // XRONOS_TELEMETRY_OTEL_OTEL_RUNTIME_DATA_LOGGER_HH
