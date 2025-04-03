// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_ASSERT_HH
#define XRONOS_RUNTIME_ASSERT_HH

#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/gen/config.hh"

#include <cassert>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef RUNTIME_VALIDATE
constexpr bool runtime_validation = true;
#else
constexpr bool runtime_validation = false;
#endif

#ifdef NDEBUG
constexpr bool runtime_assertion = false;
#else
constexpr bool runtime_assertion = true;
#endif

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define reactor_assert(x) assert(x)

#ifdef RUNTIME_USE_BACKTRACE

// NOLINTNEXTLINE(llvm-include-order)
#include RUNTIME_BACKTRACE_HEADER
#include <array>
#include <iostream>

namespace xronos::runtime {

constexpr std::size_t MAX_TRACE_SIZE{16};

inline void print_backtrace() {
  std::array<void*, MAX_TRACE_SIZE> trace{nullptr};
  int size = backtrace(trace.data(), MAX_TRACE_SIZE);
  char** messages = backtrace_symbols(trace.data(), size);
  for (int i{0}; i < size; i++) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    std::cerr << "[backtrace] " << messages[i] << '\n';
  }
}

} // namespace xronos::runtime
#else
namespace xronos::runtime {
inline void print_backtrace() {}
} // namespace xronos::runtime
#endif // XRONOS_RUNTIME_BACKTRACE_SUPPORT

namespace xronos::runtime {

class ValidationError : public std::runtime_error {

public:
  explicit ValidationError(const std::string_view msg)
      : std::runtime_error(std::string(msg)) {}
};

constexpr void validate([[maybe_unused]] bool condition, [[maybe_unused]] const std::string_view message) {
  if constexpr (runtime_validation) {
    if (!condition) {
      print_backtrace();
      throw ValidationError(message);
    }
  }
}

template <typename E> constexpr auto extract_value(E enum_value) -> typename std::underlying_type_t<E> {
  return static_cast<typename std::underlying_type_t<E>>(enum_value);
}

void assert_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] Phase phase);

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_ASSERT_HH
