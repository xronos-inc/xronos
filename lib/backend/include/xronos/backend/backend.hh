// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_BACKEND_BACKEND_HH
#define XRONOS_BACKEND_BACKEND_HH

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/backend/engine.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::backend {

using RuntimeFactory = std::function<std::unique_ptr<runtime::Runtime>()>;

class Backend {
public:
  // Creates a backend that executes programs with the runtime produced by
  // `runtime_factory`. The factory is invoked inside `run`.
  explicit Backend(RuntimeFactory runtime_factory);
  ~Backend();
  Backend(Backend&&) noexcept;
  auto operator=(Backend&&) noexcept -> Backend&;
  Backend(const Backend&) = delete;
  auto operator=(const Backend&) = delete;

  // The backend's frozen cross-boundary view. Stable under moves of this
  // object and valid until it is destroyed.
  [[nodiscard]] auto abi() const noexcept -> abi::Backend&;

  // Replaces the runtime factory. Must be called before `run`.
  void set_runtime_factory(RuntimeFactory runtime_factory);

  // Enables telemetry for the eventual run. A no-op at run time when the
  // library was built without telemetry support. Must be called before
  // `run`.
  void enable_telemetry(std::string_view application_name, std::string_view endpoint);

  // Completes assembly of the program. May only be called once, after all
  // elements are registered.
  void assemble();

  // Checks the assembled program and returns error messages. An empty
  // result means the model is valid. Requires `assemble` (throws
  // `std::logic_error` otherwise). May be called again after further
  // changes. A verdict covers the model exactly as checked, so any later
  // change outdates it and `run` requires a fresh passing call.
  [[nodiscard]] auto validate() -> std::vector<std::string>;

  // Exports the diagram of the assembled model. Requires `assemble` (throws
  // `std::logic_error` otherwise); a no-op when the library was built
  // without diagram support.
  void export_diagram();

  // Runs the program to completion (blocking). Requires `assemble` and a
  // `validate` that returned empty on the model as it stands now (throws
  // `std::logic_error` otherwise) and may only be called once.
  void run(const runtime::ExecutionProperties& properties);

private:
  // The engine sits behind a pointer, which is what keeps the `abi::Backend`
  // view stable while this handle moves.
  std::unique_ptr<Engine> engine_;
  RuntimeFactory runtime_factory_;
};

} // namespace xronos::backend

#endif // XRONOS_BACKEND_BACKEND_HH
