// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/time.hh"

#include <chrono>
#include <ostream>

#include "xronos/util/time_operators.hh"

namespace xronos::sdk::inline operators {

auto operator<<(std::ostream& os, TimePoint tp) -> std::ostream& { return util::operators::operator<<(os, tp); }

auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream& {
  return util::operators::operator<<(os, dur);
}

auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream& {
  return util::operators::operator<<(os, dur);
}

auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream& {
  return util::operators::operator<<(os, dur);
}

auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream& {
  return util::operators::operator<<(os, dur);
}

} // namespace xronos::sdk::inline operators
