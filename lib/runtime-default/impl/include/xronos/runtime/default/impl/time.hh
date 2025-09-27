// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_IMPL_TIME_HH
#define XRONOS_RUNTIME_DEFAULT_IMPL_TIME_HH

#include <chrono>

namespace xronos::runtime::default_::impl {

using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

auto inline get_physical_time() -> TimePoint { return std::chrono::system_clock::now(); }

} // namespace xronos::runtime::default_::impl

#endif
