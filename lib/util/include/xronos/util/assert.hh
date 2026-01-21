// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_UTIL_ASSERT_HH
#define XRONOS_UTIL_ASSERT_HH

#include <cstdlib>
#include <source_location>
#include <string_view>

#include "xronos/util/logging.hh"

namespace xronos::util {

namespace detail {

#if !defined(NDEBUG) || defined(__clang_analyzer__)
constexpr bool assert_enabled{true};
#else
constexpr bool assert_enabled{false};
#endif

} // namespace detail

inline void assert_(bool condition, std::string_view message = "",
                    const std::source_location& location = std::source_location::current()) {
  if constexpr (detail::assert_enabled) {
    static constexpr std::string_view failed{"Assertion failed!"};
    if (!condition) {
      if (message.empty()) {
        log::error(location) << failed;
      } else {
        log::error(location) << failed << ' ' << message;
      }
      std::abort();
    }
  }
}

inline void not_implemented(const std::source_location& location = std::source_location::current()) {
  assert_(false, "Called a function that is not implemented!", location);
}

} // namespace xronos::util

#endif // XRONOS_UTIL_ASSERT_HH
