// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_TIME_HH
#define XRONOS_SDK_TIME_HH

#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <mutex>
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
inline auto operator<<(std::ostream& os, TimePoint tp) -> std::ostream& {
  constexpr std::size_t nanosecond_digits{9};
  constexpr std::uint64_t nanoseconds_per_second{1'000'000'000UL};

  // print time down to the second
  std::time_t time =
      std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::system_clock::duration>(tp));
  {
    static std::mutex mutex{};
    const std::lock_guard<std::mutex> lock{mutex};
    // std::localtime may not be thread safe and we protect it with the lock above.
    // It also returns nullptr for a time_t it cannot represent as a calendar date.
    // A valid nanosecond TimePoint stays in range on a 64-bit time_t (the whole
    // range is years 1677-2262), so this is defensive -- but guard rather than
    // dereference a possible null.
    const std::tm* tm_ptr = std::localtime(&time);
    if (tm_ptr == nullptr) {
      return os << tp.time_since_epoch().count() << "ns";
    }
    const std::tm local_time = *tm_ptr;
    os << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
  }

  // also print nanoseconds
  std::uint64_t nanoseconds = tp.time_since_epoch().count() % nanoseconds_per_second;
  os << '.' << std::setw(nanosecond_digits) << std::setfill('0') << nanoseconds;

  return os;
}

/**
 * Write a duration in seconds including the unit to an ostream.
 */
inline auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream& { return os << dur.count() << 's'; }

/**
 * Write a duration in milliseconds including the unit to an ostream.
 */
inline auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream& {
  return os << dur.count() << "ms";
}

/**
 * Write a duration in microseconds including the unit to an ostream.
 */
inline auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream& {
  return os << dur.count() << "us";
}

/**
 * Write a duration in nanoseconds including the unit to an ostream.
 */
inline auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream& {
  return os << dur.count() << "ns";
}

} // namespace operators

} // namespace xronos::sdk

#endif // XRONOS_SDK_TIME_HH
