// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_TIME_HH
#define XRONOS_CORE_TIME_HH

#include <chrono>

namespace xronos::core {

using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

} // namespace xronos::core

#endif // XRONOS_CORE_TIME_HH
