// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_UTIL_LOGGING_HH
#define XRONOS_UTIL_LOGGING_HH

#include <functional>
#include <iostream>
#include <mutex>
#include <source_location>
#include <string_view>
#include <utility>

#include "xronos/util/gen/config.hh"

namespace xronos::util::log {

namespace detail {

#ifdef NDEBUG
constexpr bool print_source_location{false};
#else
constexpr bool print_source_location{true};
#endif

class Logger {
public:
  Logger(std::string_view prefix, std::source_location location)
      : location_{location} {
    ostream_.get() << prefix;
  }
  ~Logger() {
    if (lock_.owns_lock()) {
      // Print newline and (optionally) source location only if we are still
      // holding the lock. If this logger does not hold the lock, it means that
      // it was moved away from, and we'll let the other logger terminate the
      // line.
      if constexpr (print_source_location) {
        ostream_.get() << " (" << location_.function_name() << ", " << location_.file_name() << ":" << location_.line()
                       << ')';
      }
      ostream_.get() << '\n';
    }
  }
  Logger(const Logger&) = delete;
  Logger(Logger&&) = default;
  auto operator=(const Logger&) = delete;
  auto operator=(Logger&&) -> Logger& = default;

  template <class T> auto operator<<(T&& arg) -> Logger& {
    ostream_.get() << std::forward<T>(arg); // NOLINT
    return *this;
  }

  static void set_ostream(std::ostream& ostream) {
    std::lock_guard<std::mutex> lock{mutex_};
    ostream_ = ostream;
  }

private:
  static inline std::mutex mutex_;
  static inline std::reference_wrapper<std::ostream> ostream_{std::cerr}; // NOLINT

  std::unique_lock<std::mutex> lock_{mutex_};
  std::source_location location_;
};

struct DisabledLogger {
  DisabledLogger() = default;
  DisabledLogger(const DisabledLogger&) = delete;
  DisabledLogger(DisabledLogger&&) = default;
  auto operator=(const DisabledLogger&) = delete;
  auto operator=(DisabledLogger&&) -> DisabledLogger& = default;
  ~DisabledLogger() = default;
  template <class T> auto operator<<([[maybe_unused]] T&& arg) -> DisabledLogger& { return *this; }
};

} // namespace detail

constexpr bool debug_enabled = 4 <= XRONOS_LOG_LEVEL;
constexpr bool info_enabled = 3 <= XRONOS_LOG_LEVEL;
constexpr bool warn_enabled = 2 <= XRONOS_LOG_LEVEL;
constexpr bool error_enabled = 1 <= XRONOS_LOG_LEVEL;

inline auto debug([[maybe_unused]] std::source_location location = std::source_location::current()) noexcept -> auto {
  if constexpr (debug_enabled) {
    return detail::Logger{"[DEBUG] ", location};
  } else {
    return detail::DisabledLogger{};
  }
}

inline auto info([[maybe_unused]] std::source_location location = std::source_location::current()) noexcept -> auto {
  if constexpr (info_enabled) {
    return detail::Logger{"[INFO]  ", location};
  } else {
    return detail::DisabledLogger{};
  }
}

inline auto warn([[maybe_unused]] std::source_location location = std::source_location::current()) noexcept -> auto {
  if constexpr (warn_enabled) {
    return detail::Logger{"[WARN]  ", location};
  } else {
    return detail::DisabledLogger{};
  }
}

inline auto error([[maybe_unused]] std::source_location location = std::source_location::current()) noexcept -> auto {
  if constexpr (error_enabled) {
    return detail::Logger{"[ERROR] ", location};
  } else {
    return detail::DisabledLogger{};
  }
}

class NamedLogger {
public:
  NamedLogger(std::string_view name)
      : name_{name} {}

  [[nodiscard]] auto debug(std::source_location location = std::source_location::current()) const noexcept -> auto {
    return std::move(log::debug(location) << '(' << name_ << ") ");
  }
  [[nodiscard]] auto info(std::source_location location = std::source_location::current()) const noexcept -> auto {
    return std::move(log::info(location) << '(' << name_ << ") ");
  }
  [[nodiscard]] auto warn(std::source_location location = std::source_location::current()) const noexcept -> auto {
    return std::move(log::warn(location) << '(' << name_ << ") ");
  }
  [[nodiscard]] auto error(std::source_location location = std::source_location::current()) const noexcept -> auto {
    return std::move(log::error(location) << '(' << name_ << ") ");
  }

private:
  std::string name_;
};

} // namespace xronos::util::log

#endif // XRONOS_UTIL_LOGGING_HH
