// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_IMPL_ASSERT_HH
#define XRONOS_RUNTIME_DEFAULT_IMPL_ASSERT_HH

#include "xronos/runtime/default/impl/fwd.hh"
#include "xronos/runtime/default/impl/gen/config.hh"
#include "xronos/runtime/interfaces.hh"

#include <string_view>

#ifdef RUNTIME_VALIDATE
constexpr bool runtime_validation = true;
#else
constexpr bool runtime_validation = false;
#endif

namespace xronos::runtime::default_::impl {

constexpr void validate([[maybe_unused]] bool condition, [[maybe_unused]] std::string_view message) {
  if constexpr (runtime_validation) {
    if (!condition) {
      throw ValidationError(message);
    }
  }
}

void validate_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] Phase phase);

} // namespace xronos::runtime::default_::impl

#endif // XRONOS_RUNTIME_DEFAULT_IMPL_ASSERT_HH
