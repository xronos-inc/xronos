// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_TIME_HH
#define XRONOS_RUNTIME_TIME_HH

#include <chrono>
#include <iostream>

namespace xronos::runtime {

using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

auto inline get_physical_time() -> TimePoint { return std::chrono::system_clock::now(); }

inline namespace operators {

auto operator<<(std::ostream& os, TimePoint tp) -> std::ostream&;

auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream&;
auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream&;
auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream&;
auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream&;

} // namespace operators

} // namespace xronos::runtime

#endif
