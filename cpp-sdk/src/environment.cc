// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/environment.hh"

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/backend/backend.hh"
#include "xronos/runtime/default/default_runtime.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/gen/config.hh"
#include "xronos/sdk/runtime_provider.hh"
#include "xronos/sdk/time.hh"
#include "xronos/util/logging.hh"

namespace xronos::sdk::detail {

// The concrete program context the SDK library hands out: owns the complete
// default backend by value. It is defined only in this translation unit --
// create_default_program_context is its only constructor, and the Environment
// bodies below hold it by its concrete type to drive the backend directly (no
// downcast). Header-inline code only ever sees it upcast to the ProgramContext
// interface, through Environment::program_context().
class DefaultProgramContext final : public ProgramContext {
public:
  explicit DefaultProgramContext(backend::RuntimeFactory runtime_factory)
      : backend_{std::move(runtime_factory)} {}

  [[nodiscard]] auto backend() const noexcept -> abi::Backend& final { return backend_.abi(); }

  // The backend's full surface, for driving the lifecycle and environment-level
  // configuration across the implementation-side seam.
  [[nodiscard]] auto backend_impl() noexcept -> backend::Backend& { return backend_; }

private:
  backend::Backend backend_;
};

} // namespace xronos::sdk::detail

namespace xronos::sdk {

namespace {

// Shared by both execute() overloads so the provider overload can reject a
// repeat call before it mutates the backend.
[[noreturn]] void throw_executed_twice() {
  throw std::logic_error("Environment::execute cannot be called twice on the same environment. To correctly start a "
                         "new instance of the program, create a new environment.");
}

} // namespace

Environment::Environment(bool fast_fwd_execution, Duration timeout, bool render_reactor_graph)
    : default_program_context_{detail::create_default_program_context()}
    , program_context_{default_program_context_} // implicit, compiler-checked upcast to the interface
    , timeout_{timeout}
    , fast_fwd_execution_{fast_fwd_execution}
    , diagram_export_requested_{render_reactor_graph} {}

void Environment::execute() {
  if (std::exchange(executed_, true)) {
    throw_executed_twice();
  }

  auto& backend = default_program_context_->backend_impl();
  backend.assemble();
  // A validation failure is a programming error in the reactor program, so
  // it is fatal here: log every finding, then throw one summary exception.
  if (const auto errors = backend.validate(); !errors.empty()) {
    for (const auto& error : errors) {
      util::log::error() << error;
    }
    throw ValidationError{"The reactor program is invalid and cannot be executed."};
  }
  if (diagram_export_requested_) {
    backend.export_diagram();
  }
  // Prepares the run, publishes it behind the runtime-backend facade, and
  // blocks until the program finishes.
  backend.run(runtime::ExecutionProperties{.timeout = timeout_, .fast_mode = fast_fwd_execution_});
}

void Environment::execute(const RuntimeProvider& runtime_provider) {
  // Bind the SDK version to this translation unit's compile-time value. config::VERSION is an
  // inline constexpr in a public header; reading it through a local constexpr guarantees the
  // comparison reflects the version baked into *this* SDK library and cannot be affected by
  // symbol interposition of config::VERSION across shared objects on ELF platforms.
  constexpr std::string_view sdk_version = config::VERSION;
  if (runtime_provider.version() != sdk_version) {
    throw VersionMismatchError("Xronos SDK/runtime version mismatch: the SDK is version " + std::string{sdk_version} +
                               " but the runtime is version " + std::string{runtime_provider.version()} +
                               ". The SDK and the runtime must be built from the same release.");
  }
  // Reject a repeat call before touching the backend: set_runtime_factory below
  // would otherwise rebind the factory on a call that then throws from execute().
  if (executed_) {
    throw_executed_twice();
  }
  // Rebind the backend's runtime factory to the caller's provider, then run the
  // standard lifecycle. Only the parameterless execute() drives assembly and the
  // run; this overload just selects the runtime beforehand.
  default_program_context_->backend_impl().set_runtime_factory(
      [&runtime_provider]() { return runtime_provider.get_runtime(); });
  execute();
}

void Environment::enable_telemetry(std::string_view application_name, std::string_view endpoint) {
  default_program_context_->backend_impl().enable_telemetry(application_name, endpoint);
}

namespace detail {

// Declared in environment.hh; the inline Environment constructors call it to
// obtain a program context from the linked SDK library.
auto create_default_program_context() -> std::shared_ptr<DefaultProgramContext> {
  return std::make_shared<DefaultProgramContext>(
      []() { return std::make_unique<runtime::default_::DefaultRuntime>(); });
}

} // namespace detail

} // namespace xronos::sdk
