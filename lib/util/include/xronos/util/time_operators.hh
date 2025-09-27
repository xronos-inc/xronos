// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_UTIL_TIME_OPERATORS_HH
#define XRONOS_UTIL_TIME_OPERATORS_HH

#include <chrono>
#include <ostream>

namespace xronos::util::inline operators {

auto operator<<(std::ostream& os, std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp)
    -> std::ostream&;
inline auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream& { return os << dur.count() << 's'; }
inline auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream& {
  return os << dur.count() << "ms";
}
inline auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream& {
  return os << dur.count() << "us";
}
inline auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream& {
  return os << dur.count() << "ns";
}

} // namespace xronos::util::inline operators

#endif // XRONOS_UTIL_TIME_OPERATORS_HH
