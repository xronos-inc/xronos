// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_LOGGING_HH
#define XRONOS_RUNTIME_LOGGING_HH

#include <iostream>
#include <mutex>
#include <string>

#include "xronos/runtime/gen/config.hh"

namespace xronos::runtime::log {

template <bool enabled> class BaseLogger {};

template <> class BaseLogger<true> {
private:
  using Lock = std::unique_lock<std::mutex>;

  const std::string log_prefix_{};
  inline static std::mutex mutex_{};
  Lock lock_{};

public:
  explicit BaseLogger(const std::string& log_prefix)
      : log_prefix_(log_prefix)
      , lock_(mutex_) {
    std::cerr << log_prefix;
  }
  BaseLogger(const BaseLogger&) = delete;
  auto operator=(const BaseLogger&) -> BaseLogger& = delete;
  BaseLogger(BaseLogger&&) = delete;
  auto operator=(BaseLogger&&) -> BaseLogger& = delete;

  template <class T> auto operator<<(const T& msg) -> BaseLogger& {
    // FIXME
    std::cerr << msg; // NOLINT
    return *this;
  }

  ~BaseLogger() { std::cerr << '\n'; }
};

template <> class BaseLogger<false> {
public:
  explicit BaseLogger([[maybe_unused]] const std::string& /*unused*/) {}
  BaseLogger(const BaseLogger&) = delete;
  auto operator=(const BaseLogger&) -> BaseLogger& = delete;
  BaseLogger(BaseLogger&&) = delete;
  auto operator=(BaseLogger&&) -> BaseLogger& = delete;

  template <class T> auto operator<<([[maybe_unused]] const T& /*unused*/) const -> const BaseLogger& { return *this; }

  ~BaseLogger() = default;
};

constexpr bool debug_enabled = 4 <= RUNTIME_LOG_LEVEL;
constexpr bool info_enabled = 3 <= RUNTIME_LOG_LEVEL;
constexpr bool warn_enabled = 2 <= RUNTIME_LOG_LEVEL;
constexpr bool error_enabled = 1 <= RUNTIME_LOG_LEVEL;

struct Debug : BaseLogger<debug_enabled> {
  Debug()
      : BaseLogger<debug_enabled>("[DEBUG] ") {}
};
struct Info : BaseLogger<info_enabled> {
  Info()
      : BaseLogger<info_enabled>("[INFO]  ") {}
};
struct Warn : BaseLogger<warn_enabled> {
  Warn()
      : BaseLogger<warn_enabled>("[WARN]  ") {}
};
struct Error : BaseLogger<error_enabled> {
  Error()
      : BaseLogger<error_enabled>("[ERROR] ") {}
};

class NamedLogger {
private:
  std::string debug_prefix_{};
  std::string info_prefix_{};
  std::string warn_prefix_{};
  std::string error_prefix_{};

public:
  NamedLogger(const std::string& name)
      : debug_prefix_("[DEBUG] (" + name + ") ")
      , info_prefix_("[INFO]  (" + name + ") ")
      , warn_prefix_("[WARN]  (" + name + ") ")
      , error_prefix_("[ERROR] (" + name + ") ") {}

  [[nodiscard]] auto debug() const -> BaseLogger<debug_enabled> { return BaseLogger<debug_enabled>(debug_prefix_); }
  [[nodiscard]] auto info() const -> BaseLogger<info_enabled> { return BaseLogger<info_enabled>(info_prefix_); }
  [[nodiscard]] auto warn() const -> BaseLogger<warn_enabled> { return BaseLogger<warn_enabled>(warn_prefix_); }
  [[nodiscard]] auto error() const -> BaseLogger<error_enabled> { return BaseLogger<error_enabled>(error_prefix_); }
};

} // namespace xronos::runtime::log

#endif // XRONOS_RUNTIME_LOGGING_HH
