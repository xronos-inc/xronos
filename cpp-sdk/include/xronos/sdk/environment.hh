// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_ENVIRONMENT_HH
#define XRONOS_SDK_ENVIRONMENT_HH

#include <memory>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string_view>

#include "xronos/abi/exceptions.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/connect.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/runtime_provider.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

/**
 * Exception that is thrown when a program reaches an invalid state.
 */
using ValidationError = abi::ValidationError;

/**
 * Exception thrown by Environment::execute(const RuntimeProvider&) when the
 * provided runtime was built from a different release than the SDK.
 *
 * The runtime may be built and linked separately from the SDK, so a program may
 * inadvertently combine a runtime and an SDK from incompatible releases.
 */
class VersionMismatchError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

namespace detail {

// The concrete program context, defined only inside the SDK library (it owns
// the backend by value). Header-inline code may name it -- to hold a shared_ptr
// to it -- but can only reach the ProgramContext interface through its virtuals;
// the Environment constructor upcasts it to that interface for the inline glue.
class DefaultProgramContext;

// The factory for the SDK library's program context, compiled into the library.
// Returns the concrete type so Environment can drive the backend directly,
// without a downcast.
auto create_default_program_context() -> std::shared_ptr<DefaultProgramContext>;

} // namespace detail

/**
 * The entry point for assembling and executing reactor programs.
 *
 * The environment acts as an execution context for reactor programs. It manages
 * both the creation of reactors and the execution of reactor programs.
 */
class Environment {
public:
  /**
   * Constructor.
   */
  Environment()
      : Environment{false, Duration::max(), true} {}

  /**
   * Destructor.
   */
  ~Environment() = default;

  // The Environment may not be moved or copied: the transient context objects
  // reference the program context it owns, so a move would dangle them, and
  // there is no meaningful copy of a program's single execution.
  Environment(Environment&&) = delete;
  Environment(const Environment&) = delete;
  auto operator=(Environment&&) = delete;
  auto operator=(const Environment&) = delete;

  /**
   * Execute the reactor program.
   *
   * Initiates the execution of a reactor program. This first assembles the
   * reactor program by calling Reactor::assemble() on each reactor and then
   * initiates execution by triggering all Startup event sources.
   *
   * Returns when the reactor program terminates. The reactor program terminates
   * when there are no more events, after calling
   * ShutdownEffect::trigger_shutdown(), or when it reaches the timeout
   * configured on the environment.
   *
   * An exception thrown by a reaction handler also terminates the program.
   * The runtime then stops invoking reaction handlers; a handler that is
   * about to start may still run. Shutdown reactions run in any case, and
   * execute() rethrows the first exception.
   *
   * The program executes on the default runtime. To select a different
   * runtime, use the overload that takes a RuntimeProvider.
   */
  void execute();

  /**
   * @overload
   *
   * @param runtime_provider Provider of the runtime that should be used for
   * execution.
   */
  void execute(const RuntimeProvider& runtime_provider);

  /**
   * Get a context object for constructing top-level reactors.
   *
   * @param source_location Source location of the call site. Normally this
   * should be omitted to use the default argument.
   * @returns This environment's context.
   */
  [[nodiscard]] auto context(std::source_location source_location = std::source_location::current()) noexcept
      -> EnvironmentContext {
    return context(detail::SourceLocationView::from_std(source_location));
  }

  /**
   * @internal
   */
  [[nodiscard]] auto context(detail::SourceLocationView source_location) noexcept -> EnvironmentContext {
    return detail::ContextAccess::create_environment_context(program_context_, source_location);
  }

  /**
   * Connect two ports.
   *
   * Creates a new connection from the port given in @p from to the port given
   * in @p to. Messages are delivered without a delay. This means that the
   * timestamp at which the message is received is the same as the timestamp at
   * which it was sent.
   *
   * @tparam T Value type associated with events relayed by the connection.
   * @param from The port to draw the connection from
   * @param to The port to draw the connection to.
   */
  template <class T, template <class> class FromSerializer, template <class> class ToSerializer>
    requires std::is_same_v<FromSerializer<T>, NoSerializer<T>> || std::is_same_v<ToSerializer<T>, NoSerializer<T>> ||
             std::is_same_v<FromSerializer<T>, ToSerializer<T>>
  void connect(const InputPort<T, FromSerializer>& from, const InputPort<T, ToSerializer>& to) {
    detail::connect_impl(*program_context_, from, to, std::nullopt);
  }
  /**
   * @overload
   */
  template <class T, template <class> class FromSerializer, template <class> class ToSerializer>
    requires std::is_same_v<FromSerializer<T>, NoSerializer<T>> || std::is_same_v<ToSerializer<T>, NoSerializer<T>> ||
             std::is_same_v<FromSerializer<T>, ToSerializer<T>>
  void connect(const OutputPort<T, FromSerializer>& from, const OutputPort<T, ToSerializer>& to) {
    detail::connect_impl(*program_context_, from, to, std::nullopt);
  }
  /**
   * @overload
   */
  template <class T, template <class> class FromSerializer, template <class> class ToSerializer>
    requires std::is_same_v<FromSerializer<T>, NoSerializer<T>> || std::is_same_v<ToSerializer<T>, NoSerializer<T>> ||
             std::is_same_v<FromSerializer<T>, ToSerializer<T>>
  void connect(const OutputPort<T, FromSerializer>& from, const InputPort<T, ToSerializer>& to) {
    detail::connect_impl(*program_context_, from, to, std::nullopt);
  }

  /**
   * Connect two ports with a delay.
   *
   * Creates a new connection from the port given in @p from to the port given
   * in @p to. Messages are delivered with a delay. This means that the
   * timestamp at which the message is received is the timestamp at which it was
   * sent plus @p delay.
   *
   * @tparam T Value type associated with events relayed by the connection.
   * @param from The port to draw the connection from
   * @param to The port to draw the connection to.
   * @param delay The delay to apply to all messages.
   */
  template <class T, template <class> class FromSerializer, template <class> class ToSerializer>
    requires std::is_same_v<FromSerializer<T>, NoSerializer<T>> || std::is_same_v<ToSerializer<T>, NoSerializer<T>> ||
             std::is_same_v<FromSerializer<T>, ToSerializer<T>>
  void connect(const InputPort<T, FromSerializer>& from, const InputPort<T, ToSerializer>& to, Duration delay) {
    detail::connect_impl(*program_context_, from, to, delay);
  }
  /**
   * @overload
   */
  template <class T, template <class> class FromSerializer, template <class> class ToSerializer>
    requires std::is_same_v<FromSerializer<T>, NoSerializer<T>> || std::is_same_v<ToSerializer<T>, NoSerializer<T>> ||
             std::is_same_v<FromSerializer<T>, ToSerializer<T>>
  void connect(const OutputPort<T, FromSerializer>& from, const OutputPort<T, ToSerializer>& to, Duration delay) {
    detail::connect_impl(*program_context_, from, to, delay);
  }
  /**
   * @overload
   */
  template <class T, template <class> class FromSerializer, template <class> class ToSerializer>
    requires std::is_same_v<FromSerializer<T>, NoSerializer<T>> || std::is_same_v<ToSerializer<T>, NoSerializer<T>> ||
             std::is_same_v<FromSerializer<T>, ToSerializer<T>>
  void connect(const OutputPort<T, FromSerializer>& from, const InputPort<T, ToSerializer>& to, Duration delay) {
    detail::connect_impl(*program_context_, from, to, delay);
  }

  /**
   * @brief Enable collecting and sending telemetry data from the application.
   *
   * See <a href="../../telemetry.html">Telemetry</a> and
   * <a href="../../dashboard.html">Dashboard</a> for more information on
   * producing, collecting and visualizing telemetry data.
   *
   * @param application_name: The name of the application as it should appear
   *        in the telemetry metadata.
   * @param endpoint: The network endpoint to send telemetry data to. This is
   *        typically port 4317 on the host running the
   *        <a href="../../dashboard.html">Dashboard</a>.
   */
  void enable_telemetry(std::string_view application_name = "xronos", std::string_view endpoint = "localhost:4317");

protected:
  /**
   * @internal
   * @brief Low-level constructor for the environment that supports advanced configuration.
   *
   * @details This constructor usually should not be called directly.
   * @param fast_fwd_execution Use a special mode of execution that skips waiting between
   * executing events and instead processes events as fast as possible.
   * @param timeout The maximum amount of time to simulate before terminating.
   * @param render_reactor_graph Whether to export the reactor graph to a diagram server.
   */
  // Defined in the SDK library, where DefaultProgramContext is complete: it
  // obtains the concrete context from the factory and upcasts it to seed
  // program_context_ (an ordinary, compiler-checked base conversion).
  Environment(bool fast_fwd_execution, Duration timeout, bool render_reactor_graph);

private:
  // The program context is held twice, as the same object: default_program_context_
  // is the concrete type the compiled Environment methods drive the backend
  // through (no downcast); program_context_ is that pointer upcast to the
  // ProgramContext interface, the stable handle the header-inline glue (context
  // creation and connect) shares by reference with every Element.
  std::shared_ptr<detail::DefaultProgramContext> default_program_context_;
  std::shared_ptr<detail::ProgramContext> program_context_;

  Duration timeout_;
  bool fast_fwd_execution_;
  bool diagram_export_requested_;
  bool executed_{false};
};

/**
 * A variant of the environment that is configured for testing.
 *
 * Uses a special mode of execution that skips waiting between executing events
 * and instead processes events as fast as possible.
 */
class TestEnvironment : public Environment {
public:
  /**
   * Constructor.
   *
   * @param timeout The maximum amount of time to simulate before terminating.
   */
  TestEnvironment(Duration timeout = Duration::max())
      : Environment{true, timeout, false} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_ENVIRONMENT_HH
