// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/metric.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/telemetry/metric.hh"
#include <cstdint>

namespace xronos::sdk {

Metric::Metric(std::string_view name, ReactorContext context, std::string_view description, std::string_view unit)
    : Element(std::make_unique<telemetry::Metric>(
                  name, detail::get_reactor_instance(context),
                  detail::get_metric_data_logger_provider(detail::get_environment(context)), description, unit),
              context) {}

void Metric::record(double value) noexcept { detail::get_runtime_instance<telemetry::Metric>(*this).record(value); }
void Metric::record(std::int64_t value) noexcept {
  detail::get_runtime_instance<telemetry::Metric>(*this).record(value);
}

} // namespace xronos::sdk
