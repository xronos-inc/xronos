// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/environment.hh"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <ranges>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>

#include "xronos/runtime/assert.hh"
#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/port.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/gen/config.hh"
#include "xronos/sdk/time.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/telemetry.hh"

namespace xronos::sdk {

namespace detail {

void runtime_connect(Environment& environment, const Element& from_port, const Element& to_port) {
  auto& runtime_environment = get_environment_instance(environment);
  try {
    runtime_environment.draw_connection(detail::get_runtime_instance<runtime::Port>(from_port),
                                        detail::get_runtime_instance<runtime::Port>(to_port), {});
  } catch (const runtime::ValidationError& e) {
    throw ValidationError(e.what());
  }
}

void runtime_connect(Environment& environment, const Element& from_port, const Element& to_port, Duration delay) {
  auto& runtime_environment = get_environment_instance(environment);
  try {
    runtime_environment.draw_connection(detail::get_runtime_instance<runtime::Port>(from_port),
                                        detail::get_runtime_instance<runtime::Port>(to_port),
                                        {.type_ = runtime::ConnectionType::Delayed, .delay_ = delay});
  } catch (const runtime::ValidationError& e) {
    throw ValidationError(e.what());
  }
}

} // namespace detail

Environment::Environment()
    : Environment{std::thread::hardware_concurrency(), false, Duration::max(), true} {}

Environment::Environment(unsigned num_workers, bool fast_fwd_execution, Duration timeout, bool render_reactor_graph)
    : runtime_environment_{std::make_unique<runtime::Environment>(num_workers, fast_fwd_execution, timeout)}
    , attribute_manager_{std::make_unique<telemetry::AttributeManager>()}
    , metric_data_logger_provider_{std::make_unique<telemetry::MetricDataLoggerProvider>()}
    , render_reactor_graph_{render_reactor_graph} {}

Environment::~Environment() = default;

void Environment::execute() {
  if (has_started_execute_) {
    throw std::logic_error("Environment::execute cannot be called twice on the same environment. To correctly start a "
                           "new instance of the program, create a new environment.");
  }
  has_started_execute_ = true;
  if (telemetry_backend_ != nullptr) {
    telemetry_backend_->initialize();
  }

  std::optional<std::thread> startup_thread;
  try {
    runtime_environment_->assemble();
    startup_thread = runtime_environment_->startup();
    if constexpr (config::DIAGRAMS_ENABLED) {
      if (render_reactor_graph_) {
        detail::send_reactor_graph(*runtime_environment_, *attribute_manager_, source_locations_);
      }
    }
  } catch (const std::exception& e) {
    runtime_environment_->set_exception(); // exception will be re-thrown after cleanup
  }

  if (startup_thread.has_value()) {
    startup_thread->join();
  }

  if (telemetry_backend_ != nullptr) {
    telemetry_backend_->shutdown();
  }

  try {
    runtime_environment_->rethrow_exception_if_any();
  } catch (const runtime::ValidationError& e) {
    throw ValidationError(e.what());
  }
}

void Environment::request_shutdown() { runtime_environment_->async_shutdown(); }

auto Environment::context(std::source_location source_location) noexcept -> EnvironmentContext {
  return detail::create_context(*this, detail::SourceLocationView::from_std(source_location));
}

void Environment::enable_telemetry(std::string_view application_name, std::string_view endpoint) {
  reactor_assert(telemetry_backend_ == nullptr);
  if constexpr (config::TELEMETRY_ENABLED) {
    telemetry_backend_ = detail::create_telemetry_backend(*attribute_manager_, application_name, endpoint);
    runtime_environment_->set_data_logger(telemetry_backend_->runtime_data_logger());
    metric_data_logger_provider_->set_logger(telemetry_backend_->metric_data_logger());
  }
}

namespace detail {

[[nodiscard]] auto get_environment_instance(Environment& environment) -> runtime::Environment& {
  return *environment.runtime_environment_;
}

void store_source_location(Environment& environment, std::uint64_t uid, std::string_view fqn,
                           SourceLocationView source_location) {
  [[maybe_unused]] auto res = environment.source_locations_.try_emplace(uid, std::make_pair(fqn, source_location));
  assert(res.second);
}

auto get_attribute_manager(Environment& environment) noexcept -> telemetry::AttributeManager& {
  reactor_assert(environment.attribute_manager_ != nullptr);
  return *environment.attribute_manager_;
}

auto get_metric_data_logger_provider(Environment& environment) noexcept -> telemetry::MetricDataLoggerProvider& {
  return *environment.metric_data_logger_provider_;
}

} // namespace detail

} // namespace xronos::sdk

#ifdef XRONOS_SDK_ENABLE_TELEMETRY

#include "xronos/telemetry/otel/otel_telemetry_backend.hh"

namespace xronos::sdk::detail {

auto create_telemetry_backend(telemetry::AttributeManager& attribute_manager, std::string_view application_name,
                              std::string_view endpoint) -> std::unique_ptr<telemetry::TelemetryBackend> {
  constexpr std::size_t hostname_buffer_size = 128;
  std::array<char, hostname_buffer_size> hostname_buffer{};
  gethostname(hostname_buffer.data(), hostname_buffer_size);

  return std::make_unique<telemetry::otel::OtelTelemetryBackend>(attribute_manager, application_name, endpoint,
                                                                 std::string{hostname_buffer.data()}, getpid());
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_ENABLE_TELEMETRY

#ifdef XRONOS_SDK_ENABLE_DIAGRAMS

#include "xronos/graph_exporter/exporter.hh"
#include "xronos/messages/source_info.pb.h"

namespace xronos::sdk::detail {

void serialize_fqn(std::string_view fqn, messages::source_info::ElementSourceInfo& source_info) {
  std::size_t start = 0;
  std::size_t end = 0;
  while ((end = fqn.find('.', start)) != std::string_view::npos) {
    source_info.add_fqn(std::string(fqn.substr(start, end - start)));
    start = end + 1;
  }
  source_info.add_fqn(std::string(fqn.substr(start)));
}

void serialize_source_locations(
    const std::unordered_map<std::uint64_t, std::pair<std::string, SourceLocation>>& source_locations,
    messages::source_info::SourceInfo& source_infos) {
  for (const auto& [uid, value] : source_locations) {
    auto* source_info = source_infos.add_infos();
    const auto& [fqn, source_location] = value;
    source_info->set_uid(uid);
    serialize_fqn(fqn, *source_info);

    auto* frame = source_info->mutable_frame();
    frame->set_file(source_location.file);
    frame->set_function(source_location.function);
    frame->set_lineno(source_location.start_line);
    frame->set_end_lineno(source_location.end_line);
    frame->set_col_offset(source_location.start_column);
    frame->set_end_col_offset(source_location.end_column);
  }
}

void send_reactor_graph(
    const runtime::Environment& runtime_environment, const telemetry::AttributeManager& attribute_manager,
    const std::unordered_map<std::uint64_t, std::pair<std::string, SourceLocation>>& source_locations) {
  xronos::messages::source_info::SourceInfo source_infos;
  detail::serialize_source_locations(source_locations, source_infos);
  graph_exporter::send_reactor_graph_to_diagram_server(runtime_environment, source_infos, attribute_manager);
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_ENABLE_DIAGRAMS
