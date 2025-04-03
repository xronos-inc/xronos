// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Type aliases related to time.
 */

#ifndef XRONOS_SDK_TIME_HH
#define XRONOS_SDK_TIME_HH

#include "xronos/runtime/time.hh"

namespace xronos::sdk {
/**
 * @typedef Duration
 *
 * @brief Alias corresponding to the equivalent `std::chrono::nanoseconds` type.
 */
using Duration = runtime::Duration;
/**
 * @typedef TimePoint
 *
 * @brief Alias corresponding to the equivalent `std::chrono::time_point` type.
 */
using TimePoint = runtime::TimePoint;

inline namespace operators {
using runtime::operators::operator<<;
} // namespace operators

} // namespace xronos::sdk

#endif // XRONOS_SDK_TIME_HH
