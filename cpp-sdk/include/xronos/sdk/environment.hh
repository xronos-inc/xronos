// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_ENVIRONMENT_HH
#define XRONOS_SDK_ENVIRONMENT_HH

#include <cstdint>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

namespace detail {

auto get_environment_instance(Environment& environment) -> runtime::Environment&;
void store_source_location(Environment& environment, std::uint64_t uid, std::string_view fqn,
                           detail::SourceLocationView source_location);
auto get_attribute_manager(Environment& environment) noexcept -> telemetry::AttributeManager&;
auto get_metric_data_logger_provider(Environment& environment) noexcept -> telemetry::MetricDataLoggerProvider&;

void runtime_connect(Environment& environment, const Element& from_port, const Element& to_port);
void runtime_connect(Environment& environment, const Element& from_port, const Element& to_port, Duration delay);

auto create_telemetry_backend(telemetry::AttributeManager& attribute_manager, std::string_view application_name,
                              std::string_view endpoint) -> std::unique_ptr<telemetry::TelemetryBackend>;
void send_reactor_graph(
    const runtime::Environment& environment, const telemetry::AttributeManager& attribute_manager,
    const std::unordered_map<std::uint64_t, std::pair<std::string, SourceLocation>>& source_locations);

} // namespace detail

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
   */
  void execute();

  /**
   * Request the termination of a currently running reactor program.
   *
   * Terminates a running program at the next convenience. After completing all
   * currently active reactions, this triggers the Shutdown event sources. Once
   * all reactions triggered by Shutdown are processed, the program terminates.
   */
  void request_shutdown();

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
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to) {
    detail::runtime_connect(*this, from, to);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to) {
    detail::runtime_connect(*this, from, to);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to) {
    detail::runtime_connect(*this, from, to);
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
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to, Duration delay) {
    detail::runtime_connect(*this, from, to, delay);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to, Duration delay) {
    detail::runtime_connect(*this, from, to, delay);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to, Duration delay) {
    detail::runtime_connect(*this, from, to, delay);
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
  std::unique_ptr<runtime::Environment> runtime_environment_;
  std::unique_ptr<telemetry::AttributeManager> attribute_manager_;
  std::unique_ptr<telemetry::MetricDataLoggerProvider> metric_data_logger_provider_;
  std::unique_ptr<telemetry::TelemetryBackend> telemetry_backend_{nullptr};

  bool render_reactor_graph_;
  bool has_started_execute_{false};

  [[nodiscard]] auto runtime_instance() noexcept -> runtime::Environment& { return *runtime_environment_; }
  [[nodiscard]] auto runtime_instance() const noexcept -> const runtime::Environment& { return *runtime_environment_; }

  std::unordered_map<std::uint64_t, std::pair<std::string, detail::SourceLocation>> source_locations_{};

  friend void detail::store_source_location(Environment& environment, std::uint64_t uid, std::string_view fqn,
                                            detail::SourceLocationView source_location);
  friend auto detail::get_environment_instance(Environment& environment) -> runtime::Environment&;
  friend auto detail::get_attribute_manager(Environment& environment) noexcept -> telemetry::AttributeManager&;
  friend auto detail::get_metric_data_logger_provider(Environment& environment) noexcept
      -> telemetry::MetricDataLoggerProvider&;
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
