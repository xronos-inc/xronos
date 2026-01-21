// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_TIME_HH
#define XRONOS_SDK_TIME_HH

#include <chrono>
#include <ostream>

namespace xronos::sdk {

/**
 * Data type used to represent durations.
 */
using Duration = std::chrono::nanoseconds;

/**
 * Data type used to represent time points.
 */
using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

/**
 * Contains convenience streaming operators that allow printing time points and durations including their units.
 */
inline namespace operators {

/**
 * Write a timepoint to an ostream.
 */
auto operator<<(std::ostream& os, TimePoint tp) -> std::ostream&;

/**
 * Write a duration in seconds including the unit to an ostream.
 */
auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream&;

/**
 * Write a duration in milliseconds including the unit to an ostream.
 */
auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream&;

/**
 * Write a duration in microseconds including the unit to an ostream.
 */
auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream&;

/**
 * Write a duration in nanoseconds including the unit to an ostream.
 */
auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream&;

} // namespace operators

} // namespace xronos::sdk

#endif // XRONOS_SDK_TIME_HH
