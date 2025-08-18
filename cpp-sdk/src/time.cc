// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/time.hh"

#include <chrono>
#include <ostream>

#include "xronos/runtime/time.hh"

namespace xronos::sdk::inline operators {

auto operator<<(std::ostream& os, TimePoint tp) -> std::ostream& { return runtime::operators::operator<<(os, tp); }

auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream& {
  return runtime::operators::operator<<(os, dur);
}

auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream& {
  return runtime::operators::operator<<(os, dur);
}

auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream& {
  return runtime::operators::operator<<(os, dur);
}

auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream& {
  return runtime::operators::operator<<(os, dur);
}

} // namespace xronos::sdk::inline operators
