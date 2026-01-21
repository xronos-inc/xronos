// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_METRIC_HH
#define XRONOS_SDK_METRIC_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

/**
 * A reactor element for recording metric data to an external data base.
 *
 * Can be used as a reaction @ref BaseReaction::MetricEffect "effect" allowing
 * the reaction handler to record values using the metric.
 */
class Metric final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the metric.
   * @param context The containing reactor's context.
   * @param description A human readable description of the metric.
   * @param unit The unit of values recorded using the metric.
   */
  Metric(std::string_view name, const ReactorContext& context, std::string_view description,
         std::string_view unit = "");

  /**
   * Get the description.
   */
  [[nodiscard]] auto description() const noexcept -> const std::string&;
  /**
   * Get the unit.
   */
  [[nodiscard]] auto unit() const noexcept -> const std::string&;

private:
  void record(double value) noexcept;
  void record(std::int64_t value) noexcept;

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_METRIC_HH
