// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `Environment` class.
 */

#ifndef XRONOS_SDK_ENVIRONMENT_HH
#define XRONOS_SDK_ENVIRONMENT_HH

#include <cstdint>
#include <memory>
#include <source_location>
#include <string_view>
#include <unordered_map>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"

#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/environment.hh"

namespace xronos::sdk {

namespace detail {

auto get_environment_instance(Environment& environment) -> runtime::Environment&;
void store_source_location(Environment& environment, std::uint64_t uid, std::string_view fqn,
                           std::source_location source_location);
auto get_attribute_manager(Environment& environment) noexcept -> telemetry::AttributeManager&;
auto get_metric_data_logger_provider(Environment& environment) noexcept -> telemetry::MetricDataLoggerProvider&;

} // namespace detail

/**
 * @brief The entry point for assembling and executing reactor programs.
 *
 * @details The environment acts as an execution context for reactor programs.
 * It manages both the creation of reactors and the execution of reactor
 * programs.
 */
class Environment {
public:
  /**
   * @brief Construct a new Environment object.
   *
   * @details Programs that use this environment will execute indefinitely
   * or until a shutdown is requested, and will run with the closest possible
   * approximation to real time.
   */
  Environment();
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
   * @brief Execute the reactor program.
   *
   * @details Initiates the execution of a reactor program. This triggers
   * `xronos::sdk::Startup` and instructs the runtime to start processing
   * reactions.
   *
   * Returns when the reactor program terminates.
   */
  void execute();
  /**
   * @brief Request the termination of a currently running reactor program.
   *
   * @details Terminates a program started with `execute` at the next
   * convenience. This triggers `xronos::sdk::Shutdown` after
   * completing all currently active reactions, and stops program execution after
   * processing all reactions triggered by `xronos::sdk::Shutdown`.
   */
  void request_shutdown();

  /**
   * @brief Get the top-level initialization context object.
   *
   * @details This is needed in order to instantiate top-level reactors.
   */
  [[nodiscard]] auto
  context(std::source_location source_location = std::source_location::current()) noexcept -> EnvironmentContext;

  /**
   * @brief Connect two ports.
   *
   * @details Creates a new connection from the port given in @p from to the
   * port given in @p to. Messages communicated by the connection are delivered
   * with no delay.
   * @tparam T The type communicated by the connection.
   * @param from The port to draw the connection from
   * @param to The port to draw the connection to.
   */
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to) {
    runtime_environment_->draw_connection(detail::get_runtime_instance<runtime::Port<T>>(from),
                                          detail::get_runtime_instance<runtime::Port<T>>(to), {});
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to) {
    runtime_environment_->draw_connection(detail::get_runtime_instance<runtime::Port<T>>(from),
                                          detail::get_runtime_instance<runtime::Port<T>>(to), {});
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to) {
    runtime_environment_->draw_connection(detail::get_runtime_instance<runtime::Port<T>>(from),
                                          detail::get_runtime_instance<runtime::Port<T>>(to), {});
  }
  /**
   * @brief Connect two ports with a delay.
   *
   * @details Creates a new connection from the port given in @p from to the
   * port given in @p to.
   * @tparam T The type communicated by the connection.
   * @param from The port to draw the connection from
   * @param to The port to draw the connection to.
   * @param delay The connection waits for @p delay before delivering the
   * messages to @p to.
   */
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to, Duration delay) {
    runtime_environment_->draw_connection(detail::get_runtime_instance<runtime::Port<T>>(from),
                                          detail::get_runtime_instance<runtime::Port<T>>(to),
                                          {runtime::ConnectionType::Delayed, delay});
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to, Duration delay) {
    runtime_environment_->draw_connection(detail::get_runtime_instance<runtime::Port<T>>(from),
                                          detail::get_runtime_instance<runtime::Port<T>>(to),
                                          {runtime::ConnectionType::Delayed, delay});
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to, Duration delay) {
    runtime_environment_->draw_connection(detail::get_runtime_instance<runtime::Port<T>>(from),
                                          detail::get_runtime_instance<runtime::Port<T>>(to),
                                          {runtime::ConnectionType::Delayed, delay});
  }

  /**
   * @brief Enable collecting and sending telemetry data from the application.
   *
   * @param application_name The name of the current application.
   * @param endpoint The endpoint to which the traces should be sent. This
   * should be a URI of the form `http://host:port` or `https://host:port`.
   */
  void enable_telemetry(std::string_view application_name = "xronos", std::string_view endpoint = "localhost:4317");

  [[deprecated("Use enable_telemetry() instead")]] void enable_tracing(std::string_view application_name = "xronos",
                                                                       std::string_view endpoint = "localhost:4317");

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
  Environment(bool fast_fwd_execution, Duration timeout, bool render_reactor_graph);

private:
  std::unique_ptr<runtime::Environment> runtime_environment_;
  std::unique_ptr<telemetry::AttributeManager> attribute_manager_;
  std::unique_ptr<telemetry::MetricDataLoggerProvider> metric_data_logger_provider_;
  std::unique_ptr<telemetry::TelemetryBackend> telemetry_backend_{nullptr};

  bool render_reactor_graph_;
  bool has_started_execute_{false};

  [[nodiscard]] auto runtime_instance() noexcept -> runtime::Environment& { return *runtime_environment_; }
  [[nodiscard]] auto runtime_instance() const noexcept -> const runtime::Environment& { return *runtime_environment_; }

  std::unordered_map<std::uint64_t, std::pair<std::string, std::source_location>> source_locations_{};

  friend void detail::store_source_location(Environment& environment, std::uint64_t uid, std::string_view fqn,
                                            std::source_location source_location);
  friend auto detail::get_environment_instance(Environment& environment) -> runtime::Environment&;
  friend auto detail::get_attribute_manager(Environment& environment) noexcept -> telemetry::AttributeManager&;
  friend auto
  detail::get_metric_data_logger_provider(Environment& environment) noexcept -> telemetry::MetricDataLoggerProvider&;
};

/**
 * @brief A variant of the environment that is configured for testing.
 *
 * @details Uses a special mode of execution that skips waiting between
 * executing events and instead processes events as fast as possible.
 */
class TestEnvironment : public Environment {
public:
  /**
   * @brief Construct a new `TestEnvironment` object.
   *
   * @param timeout The maximum amount of time to simulate before terminating.
   */
  TestEnvironment(Duration timeout = Duration::max())
      : Environment{true, timeout, false} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_ENVIRONMENT_HH
