// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cassert>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <utility>

#include "xronos/graph_exporter/exporter.hh"
#include "xronos/messages/source_info.pb.h"
#include "xronos/runtime/assert.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/time.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/otel/otel_telemetry_backend.hh"

namespace xronos::sdk {

namespace detail {

void serialize_source_locations(
    const std::unordered_map<std::uint64_t, std::pair<std::string, std::source_location>>& source_locations,
    messages::source_info::SourceInfo& source_infos);

} // namespace detail

Environment::Environment()
    : Environment{false, Duration::max(), true} {}

Environment::Environment(bool fast_fwd_execution, Duration timeout, bool render_reactor_graph)
    : runtime_environment_{std::make_unique<runtime::Environment>(std::thread::hardware_concurrency(),
                                                                  fast_fwd_execution, timeout)}
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
  runtime_environment_->assemble();
  auto thread = runtime_environment_->startup();
  if (render_reactor_graph_) {
    xronos::messages::source_info::SourceInfo source_infos;
    detail::serialize_source_locations(source_locations_, source_infos);
    graph_exporter::send_reactor_graph_to_diagram_server(*runtime_environment_, source_infos, *attribute_manager_);
  }
  thread.join();
  if (telemetry_backend_ != nullptr) {
    telemetry_backend_->shutdown();
  }
}

void Environment::request_shutdown() { runtime_environment_->async_shutdown(); }

auto Environment::context(std::source_location source_location) noexcept -> EnvironmentContext {
  return EnvironmentContext{*this, source_location};
}

void Environment::enable_tracing(std::string_view application_name, std::string_view endpoint) {
  enable_telemetry(application_name, endpoint);
};

void Environment::enable_telemetry(std::string_view application_name, std::string_view endpoint) {
  reactor_assert(telemetry_backend_ == nullptr);
  constexpr std::size_t hostname_buffer_size = 128;
  std::array<char, hostname_buffer_size> hostname_buffer{};
  gethostname(hostname_buffer.data(), hostname_buffer_size);

  telemetry_backend_ = std::make_unique<telemetry::otel::OtelTelemetryBackend>(
      *attribute_manager_, application_name, endpoint, std::string{hostname_buffer.data()}, getpid());
  runtime_environment_->set_data_logger(telemetry_backend_->runtime_data_logger());
  metric_data_logger_provider_->set_logger(telemetry_backend_->metric_data_logger());
}

namespace detail {

[[nodiscard]] auto get_environment_instance(Environment& environment) -> runtime::Environment& {
  return *environment.runtime_environment_;
}

void store_source_location(Environment& environment, std::uint64_t uid, std::string_view fqn,
                           std::source_location source_location) {
  [[maybe_unused]] auto res = environment.source_locations_.try_emplace(uid, std::make_pair(fqn, source_location));
  assert(res.second);
}

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
    const std::unordered_map<std::uint64_t, std::pair<std::string, std::source_location>>& source_locations,
    messages::source_info::SourceInfo& source_infos) {
  for (const auto& [uid, value] : source_locations) {
    auto* source_info = source_infos.add_infos();
    const auto& [fqn, source_location] = value;
    source_info->set_uid(uid);
    serialize_fqn(fqn, *source_info);

    auto* frame = source_info->mutable_frame();
    frame->set_file(source_location.file_name());
    frame->set_lineno(source_location.line());
    frame->set_end_lineno(source_location.line());
    frame->set_function(source_location.function_name());
  }
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
