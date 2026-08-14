// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_DETAIL_PROGRAM_CONTEXT_HH
#define XRONOS_SDK_DETAIL_PROGRAM_CONTEXT_HH

#include "xronos/abi/backend.hh"

namespace xronos::sdk::detail {

// The SDK's grip on the implementation behind the ABI. Elements share the
// context of their program via shared_ptr and reach the implementation
// exclusively through the `abi::Backend` it hands out; how the backend is
// owned is up to the concrete implementation of this interface (the SDK
// library's compiled internals own a complete backend, see
// Environment). The interface is deliberately minimal: everything the
// inline SDK glue needs is the backend reference.
class ProgramContext {
public:
  // Never copied or moved: Elements share it via shared_ptr.
  ProgramContext(const ProgramContext&) = delete;
  ProgramContext(ProgramContext&&) = delete;
  auto operator=(const ProgramContext&) = delete;
  auto operator=(ProgramContext&&) = delete;
  virtual ~ProgramContext() = default;

  // The implementation's assembly and lookup surface (never null).
  [[nodiscard]] virtual auto backend() const noexcept -> abi::Backend& = 0;

  // The program's hot-path lookup facade. The facade itself is always
  // valid and may be called at any time; it is the individual get_*
  // lookups on it that return nullptr until a run is prepared (see
  // abi::RuntimeBackend).
  [[nodiscard]] auto runtime_backend() const noexcept -> abi::RuntimeBackend& { return backend().runtime_backend(); }

protected:
  ProgramContext() = default;
};

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_DETAIL_PROGRAM_CONTEXT_HH
