// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/metric.hh"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "impl/xronos/sdk/detail/element.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/telemetry/metric.hh"

namespace xronos::sdk {

using CA = detail::ContextAccess;

Metric::Metric(std::string_view name, const ReactorContext& context, std::string_view description,
               std::string_view unit)
    : Element{detail::register_element(name,
                                       core::MetricTag{std::make_unique<core::MetricProperties>(core::MetricProperties{
                                           .description = std::string{description}, .unit = std::string{unit}})},
                                       context),
              context} {}

void Metric::record(double value) noexcept {
  if (program_context()->runtime_program_handle != nullptr) {
    const auto& elem = core_element();
    program_context()->metric_data_logger_provider.logger().record(
        elem, *program_context()->runtime_program_handle->get_time_access(elem.parent_uid.value()), value);
  }
}
void Metric::record(std::int64_t value) noexcept {
  if (program_context()->runtime_program_handle != nullptr) {
    const auto& elem = core_element();
    program_context()->metric_data_logger_provider.logger().record(
        elem, *program_context()->runtime_program_handle->get_time_access(elem.parent_uid.value()), value);
  }
}

auto Metric::description() const noexcept -> const std::string& {
  return std::get<core::MetricTag>(core_element().type).properties->description;
}

auto Metric::unit() const noexcept -> const std::string& {
  return std::get<core::MetricTag>(core_element().type).properties->unit;
}

} // namespace xronos::sdk
