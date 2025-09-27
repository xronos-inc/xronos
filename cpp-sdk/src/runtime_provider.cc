// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/runtime_provider.hh"

#include <memory>

#include "xronos/runtime/default/default_runtime.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::sdk {

auto DefaultRuntimeProvider::get_runtime() const noexcept -> std::unique_ptr<runtime::Runtime> {
  return std::make_unique<runtime::default_::DefaultRuntime>();
}

} // namespace xronos::sdk
