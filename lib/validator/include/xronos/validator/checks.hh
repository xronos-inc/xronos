// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_VALIDATOR_CHECKS_HH
#define XRONOS_VALIDATOR_CHECKS_HH

#include <string>
#include <vector>

#include "nonstd/expected.hpp"
#include "xronos/core/reactor_model.hh"

namespace xronos::validator {

auto run_all_checks(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>>;

auto check_shutdown_reactions(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>>;
auto check_periodic_timers(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>>;

} // namespace xronos::validator

#endif // XRONOS_VALIDATOR_CHECKS_HH
