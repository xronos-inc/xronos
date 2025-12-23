// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/util/time_operators.hh"

#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>

namespace xronos::util {

inline constexpr std::size_t NANOSECOND_DIGITS{9};
constexpr std::size_t NANOSECONDS_PER_SECOND{1'000'000'000UL};

inline namespace operators {

auto operator<<(std::ostream& os, std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp)
    -> std::ostream& {
  // print time down to the second
  std::time_t time =
      std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::system_clock::duration>(tp));
  {
    static std::mutex mutex{};
    std::lock_guard<std::mutex> lock{mutex};
    // std::localtime may not be thread safe and we protect it with the lock above
    std::tm local_time = *std::localtime(&time);
    os << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
  }

  // also print nanoseconds
  std::uint64_t nanoseconds = tp.time_since_epoch().count() % NANOSECONDS_PER_SECOND;
  os << '.' << std::setw(NANOSECOND_DIGITS) << std::setfill('0') << nanoseconds;

  return os;
};

} // namespace operators

} // namespace xronos::util
