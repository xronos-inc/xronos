// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_METRIC_HH
#define XRONOS_SDK_METRIC_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/abi/backend.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

namespace detail {

inline auto register_metric(std::string_view name, const ReactorContext& context, std::string_view description,
                            std::string_view unit) -> std::uint64_t {
  return register_with_location(context, [&]() -> std::uint64_t {
    return get_backend(context).register_metric(std::string{name}, ContextAccess::get_parent_uid(context),
                                                std::string{description}, std::string{unit});
  });
}

} // namespace detail

/**
 * A reactor element for recording numeric values to the Xronos Dashboard.
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
  Metric(std::string_view name, const ReactorContext& context, std::string_view description, std::string_view unit = "")
      : Element{detail::register_metric(name, context, description, unit), name, context}
      , description_{description}
      , unit_{unit} {}

  /**
   * Get the description.
   */
  [[nodiscard]] auto description() const noexcept -> const std::string& { return description_; }
  /**
   * Get the unit.
   */
  [[nodiscard]] auto unit() const noexcept -> const std::string& { return unit_; }

private:
  std::string description_;
  std::string unit_;
  abi::MetricRecorder* recorder_{nullptr};

  void record(double value) noexcept {
    if (auto* recorder = get_recorder(); recorder != nullptr) {
      recorder->record(value);
    }
  }
  void record(std::int64_t value) noexcept {
    if (auto* recorder = get_recorder(); recorder != nullptr) {
      recorder->record(value);
    }
  }
  [[nodiscard]] auto get_recorder() noexcept -> abi::MetricRecorder* {
    if (recorder_ == nullptr) {
      // Null until the run is prepared (see abi::RuntimeBackend).
      recorder_ = program_context()->runtime_backend().get_metric_recorder(uid());
    }
    return recorder_;
  }

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_METRIC_HH
