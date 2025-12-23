// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_ENVIRONMENT_HH
#define XRONOS_SDK_ENVIRONMENT_HH

#include <memory>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/connect.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/runtime_provider.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

/**
 * Exception that is thrown when a program reaches an invalid state.
 */
class ValidationError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

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
  Environment();

  /**
   * Destructor.
   */
  ~Environment();

  // The Environment may not be moved or copied. Copying is implicitly delete due to
  // the unique_ptr to the runtime instance. We also need to delete
  // the move constructor and assignment operator to avoid dangling references
  // in the EnvironmentContext objects.
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
   * when there are no more events, or after calling request_shutdown().
   *
   * @param runtime_provider Provider of the runtime that should be used for
   * execution. When omitted, the default runtime is used.
   */
  void execute(const RuntimeProvider& runtime_provider = DefaultRuntimeProvider{});

  /**
   * Get a context object for constructing top-level reactors.
   *
   * @param source_location Source location of the call site. Normally this
   * should be omitted to use the default argument.
   * @returns This environment's context.
   */
  [[nodiscard]] auto context(std::source_location source_location = std::source_location::current()) noexcept
      -> EnvironmentContext;

  /**
   * @internal
   */
  [[nodiscard]] auto context(detail::SourceLocationView source_location) noexcept -> EnvironmentContext;

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
   *        typically port 4137 on the host running the
   *        <a href="../../dashboard.html">Dashboard</a>.
   */
  void enable_telemetry(std::string_view application_name = "xronos", std::string_view endpoint = "localhost:4317");

protected:
  /**
   * @internal
   * @brief Low-level constructor for the environment that supports advanced configuration.
   *
   * @details This constructor usually should not be called directly.
   * @param worker Number of worker threads to be used for execution.
   * @param fast_fwd_execution Use a special mode of execution that skips waiting between
   * executing events and instead processes events as fast as possible.
   * @param timeout The maximum amount of time to simulate before terminating.
   * @param render_reactor_graph Whether to export the reactor graph to a diagram server.
   */
  Environment(unsigned workers, bool fast_fwd_execution, Duration timeout, bool render_reactor_graph);

private:
  std::shared_ptr<detail::ProgramContext> program_context_;

  unsigned num_workers_;
  Duration timeout_;
  bool fast_fwd_execution_;
  bool render_reactor_graph_;
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
      : Environment{std::thread::hardware_concurrency(), true, timeout, false} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_ENVIRONMENT_HH
