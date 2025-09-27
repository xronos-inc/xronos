// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/validator/checks.hh"

#include <string>
#include <utility>
#include <vector>

#include "nonstd/expected.hpp"
#include "xronos/core/reactor_model.hh"

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
  return detail::run_checks([&] { return check_shutdown_reactions(model); });
}

} // namespace xronos::validator
