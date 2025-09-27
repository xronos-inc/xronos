// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_RUNTIME_PROVIDER_HH
#define XRONOS_SDK_RUNTIME_PROVIDER_HH

#include <memory>

#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

/**
 * Abstract interface for providing a concrete runtime instance.
 *
 * An implementation of this interface can be passed to Environment::execute()
 * to provide a runtime for execution.
 */
struct RuntimeProvider {
  /**
   * Get a runtime instance.
   */
  [[nodiscard]] virtual auto get_runtime() const noexcept -> std::unique_ptr<runtime::Runtime> = 0;

protected:
  ~RuntimeProvider() = default;
};

/**
 * A runtime provider that provides the default runtime.
 *
 * This provider is used by Environment::execute() when no other provider is given.
 */
struct DefaultRuntimeProvider final : public RuntimeProvider {
  [[nodiscard]] auto get_runtime() const noexcept -> std::unique_ptr<runtime::Runtime> final;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_RUNTIME_PROVIDER_HH
