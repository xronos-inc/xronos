// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/validator/checks.hh"

#include <string>
#include <utility>
#include <vector>

#include "fmt/chrono.h" // IWYU pragma: keep
#include "fmt/format.h"
#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"

namespace xronos::validator {

namespace detail {

// Utility to accumulate errors
template <typename... Checks> auto run_checks(Checks&&... checks) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;

  auto process_check = [&](auto&& result) {
    if (!result) {
      error_messages.insert(error_messages.end(), result.error().begin(), result.error().end());
    }
  };

  (process_check(std::forward<Checks>(checks)()), ...); // fold-expression, calls all checks in sequence

  if (!error_messages.empty()) {
    return nonstd::unexpected(std::move(error_messages));
  }

  return {};
}

} // namespace detail

auto run_all_checks(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  return detail::run_checks([&] { return check_shutdown_reactions(model); },
                            [&] { return check_periodic_timers(model); });
}

auto check_periodic_timers(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;
  for (const auto& timer : model.element_registry.elements_of_type<core::PeriodicTimerTag>()) {
    const auto& properties = core::get_properties<core::PeriodicTimerTag>(timer);
    if (properties.period <= core::Duration::zero()) {
      error_messages.push_back(fmt::format("Timer periods must be greater than zero, but timer {} has a period of {}.",
                                           timer.fqn, properties.period));
    }
    if (properties.offset < core::Duration::zero()) {
      error_messages.push_back(fmt::format("Timer offsets must be positive, but timer {} has a period of {}.",
                                           timer.fqn, properties.offset));
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

} // namespace xronos::validator
