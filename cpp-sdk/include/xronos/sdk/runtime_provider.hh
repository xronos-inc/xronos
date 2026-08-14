// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_RUNTIME_PROVIDER_HH
#define XRONOS_SDK_RUNTIME_PROVIDER_HH

#include <memory>
#include <string_view>

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

  /**
   * Get the version of the runtime provided by this provider.
   *
   * The returned version is baked into the library that defines the provider.
   * Environment::execute() compares it against the SDK's own version to detect a
   * mismatched SDK/runtime pair (which can occur because the runtime is linked
   * separately from the SDK).
   */
  [[nodiscard]] virtual auto version() const noexcept -> std::string_view = 0;

protected:
  ~RuntimeProvider() = default;
};

/**
 * A runtime provider that provides the default runtime.
 *
 * Passing this to the Environment::execute() overload is equivalent to
 * calling the parameterless execute(), which uses the default runtime of the
 * linked backend.
 */
struct DefaultRuntimeProvider final : public RuntimeProvider {
  [[nodiscard]] auto get_runtime() const noexcept -> std::unique_ptr<runtime::Runtime> final;
  [[nodiscard]] auto version() const noexcept -> std::string_view final;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_RUNTIME_PROVIDER_HH
