// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/environment.hh"

#include <cstddef>
#include <memory>
#include <ranges>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "impl/xronos/sdk/detail/program_context.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/gen/config.hh"
#include "xronos/sdk/runtime_provider.hh"
#include "xronos/sdk/time.hh"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/telemetry.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"
#include "xronos/validator/checks.hh"

namespace xronos::sdk {

namespace detail {

auto create_telemetry_backend(const telemetry::AttributeManager& attribute_manager,
                              const core::ElementRegistry& element_registry, std::string_view application_name,
                              std::string_view endpoint) -> std::unique_ptr<telemetry::TelemetryBackend>;
void send_reactor_graph(const core::ReactorModel& model, const telemetry::AttributeManager& attribute_manager,
                        const source_location::SourceLocationRegistry& source_location_registry);

} // namespace detail

Environment::Environment()
    : Environment{std::thread::hardware_concurrency(), false, Duration::max(), true} {}

Environment::Environment(unsigned num_workers, bool fast_fwd_execution, Duration timeout, bool render_reactor_graph)
    : program_context_{std::make_shared<detail::ProgramContext>()}
    , num_workers_{num_workers}
    , timeout_{timeout}
    , fast_fwd_execution_{fast_fwd_execution}
    , render_reactor_graph_{render_reactor_graph} {}

Environment::~Environment() = default;

void Environment::execute(const RuntimeProvider& runtime_provider) {
  if (program_context_->runtime_program_handle != nullptr) {
    throw std::logic_error("Environment::execute cannot be called twice on the same environment. To correctly start a "
                           "new instance of the program, create a new environment.");
  }
  if (program_context_->telemetry_backend != nullptr) {
    program_context_->telemetry_backend->initialize();
  }

  // invoke the assemble callbacks
  for (auto& callback : program_context_->assemble_callbacks | std::views::values) {
    (callback)();
  }

  // send the reactor graph
  if constexpr (config::DIAGRAMS_ENABLED) {
    if (render_reactor_graph_) {
      detail::send_reactor_graph(program_context_->model, program_context_->attribute_manager,
                                 program_context_->source_location_registry);
    }
  }

  if (auto result = validator::run_all_checks(program_context_->model); !result.has_value()) {
    for (const auto& msg : result.error()) {
      util::log::error() << msg;
    }
    throw ValidationError{"The reactor program is invalid and cannot be executed."};
  }

  try {
    auto runtime = runtime_provider.get_runtime();
    program_context_->runtime_program_handle = runtime->initialize_reactor_program(
        program_context_->model, runtime::ExecutionProperties{.timeout = timeout_,
                                                              .num_workers = num_workers_,
                                                              .fast_mode = fast_fwd_execution_});
    // By assigning the handle first (above) and then calling execute, we make
    // sure that any reactions invoked during execute() will see a valid handle.
    // If we would call execute first (on a local variable) and then assign to
    // the context, there would be a race between the assignment and the
    // execution invoking first reactions.
    program_context_->runtime_program_handle->execute();
  } catch (const runtime::ValidationError& e) {
    throw ValidationError(e.what());
  }
}

auto Environment::context(std::source_location source_location) noexcept -> EnvironmentContext {
  return context(detail::SourceLocationView::from_std(source_location));
}

auto Environment::context(detail::SourceLocationView source_location) noexcept -> EnvironmentContext {
  return detail::ContextAccess::create_environment_context(program_context_, source_location);
}

void Environment::enable_telemetry(std::string_view application_name, std::string_view endpoint) {
  util::assert_(dynamic_cast<telemetry::NoopTelemetryBackend*>(program_context_->telemetry_backend.get()) != nullptr);
  if constexpr (config::TELEMETRY_ENABLED) {
    program_context_->telemetry_backend = detail::create_telemetry_backend(
        program_context_->attribute_manager, program_context_->model.element_registry, application_name, endpoint);
    program_context_->metric_data_logger_provider.set_logger(program_context_->telemetry_backend->metric_data_logger());
  }
}

} // namespace xronos::sdk

#ifdef XRONOS_SDK_ENABLE_TELEMETRY

#include <array>
#include <unistd.h>

#include "xronos/telemetry/otel/otel_telemetry_backend.hh"

namespace xronos::sdk::detail {

auto create_telemetry_backend(const telemetry::AttributeManager& attribute_manager,
                              const core::ElementRegistry& element_registry, std::string_view application_name,
                              std::string_view endpoint) -> std::unique_ptr<telemetry::TelemetryBackend> {
  constexpr std::size_t hostname_buffer_size = 128;
  std::array<char, hostname_buffer_size> hostname_buffer{};
  gethostname(hostname_buffer.data(), hostname_buffer_size);

  return std::make_unique<telemetry::otel::OtelTelemetryBackend>(
      attribute_manager, element_registry, application_name, endpoint, std::string{hostname_buffer.data()}, getpid());
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_ENABLE_TELEMETRY

#ifdef XRONOS_SDK_ENABLE_DIAGRAMS

#include "xronos/graph_exporter/exporter.hh"

namespace xronos::sdk::detail {

void send_reactor_graph(const core::ReactorModel& model, const telemetry::AttributeManager& attribute_manager,
                        const source_location::SourceLocationRegistry& source_location_registry) {
  graph_exporter::send_reactor_graph_to_diagram_server(model, attribute_manager, source_location_registry);
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_ENABLE_DIAGRAMS
