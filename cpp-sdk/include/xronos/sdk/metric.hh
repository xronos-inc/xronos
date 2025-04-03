// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `Metric` class.
 */

#ifndef XRONOS_SDK_METRIC_HH
#define XRONOS_SDK_METRIC_HH

#include <cstdint>
#include <string_view>

#include "xronos/sdk/element.hh"

namespace xronos::sdk {
/**
 * @brief Allows recording values to an external data base from reaction handlers.
 */
class Metric final : public Element {
public:
  /**
   * @brief Construct a new `Metric`.
   *
   * @param name The name of the `Metric`.
   * @param context The context object obtained from the `Metric`'s
   * containing reactor.
   * @param description A description of the `Metric`.
   * @param unit The unit of the `Metric`.
   */
  Metric(std::string_view name, ReactorContext context, std::string_view description, std::string_view unit = "");

  /**
   * @brief Description of the `Metric`.
   *
   * @return std::string_view The description of the `Metric`.
   */
  [[nodiscard]] auto description() const noexcept -> const std::string&;
  /**
   * @brief Unit of the `Metric`.
   *
   * @return const std::string& The unit of the `Metric`.
   */
  [[nodiscard]] auto unit() const noexcept -> const std::string&;

private:
  void record(double value) noexcept;
  void record(std::int64_t value) noexcept;

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_METRIC_HH
