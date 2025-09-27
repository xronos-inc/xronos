// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/otel/otel_telemetry_backend.hh"

#include <chrono>
#include <cstdint>
#include <ios>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string_view>
#include <utility>

#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_options.h"
#include "opentelemetry/sdk/resource/resource.h"
#include "opentelemetry/sdk/trace/batch_span_processor_factory.h"
#include "opentelemetry/sdk/trace/batch_span_processor_options.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/semconv/incubating/deployment_attributes.h"
#include "opentelemetry/semconv/incubating/host_attributes.h"
#include "opentelemetry/semconv/incubating/process_attributes.h"
#include "opentelemetry/semconv/service_attributes.h"
#include "opentelemetry/trace/noop.h"
#include "opentelemetry/trace/provider.h"

using namespace xronos::telemetry::otel;

constexpr auto BATCH_EXPORT_INTERVAL = std::chrono::milliseconds(500);

auto create_exporter(std::string_view endpoint) {
  using namespace opentelemetry::exporter::otlp;
  OtlpGrpcExporterOptions options{};
  options.endpoint = endpoint;
  options.use_ssl_credentials = endpoint.starts_with("https://");
  return OtlpGrpcExporterFactory::Create(options);
}

auto create_span_processor(std::string_view endpoint) {
  auto exporter = create_exporter(endpoint);
  using namespace opentelemetry::sdk::trace;
  BatchSpanProcessorOptions options{};
  options.schedule_delay_millis = BATCH_EXPORT_INTERVAL;
  return BatchSpanProcessorFactory::Create(std::move(exporter), options);
}

static auto get_deployment_id() -> std::string {
  auto time = std::chrono::system_clock::now();
  long time_since_epoch_ns = time.time_since_epoch() / std::chrono::nanoseconds(1);
  std::random_device platform_rng;
  // XOR with the current time in case the platform RNG does not provide the required randomness.
  // random_device's behavior is underspecified and has even been a fixed constant in at least one case:
  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=85494
  std::mt19937 rng{platform_rng() ^ static_cast<uint32_t>(time_since_epoch_ns)};
  std::uniform_int_distribution<uint64_t> dist{0, std::numeric_limits<uint64_t>::max()};
  auto random_id = dist(rng);
  std::ostringstream oss;
  oss << std::hex << random_id;
  random_id = dist(rng);
  oss << std::hex << random_id;
  return oss.str();
}

auto create_resource(std::string_view application_name, std::string_view hostname, std::int64_t pid) {
  using namespace opentelemetry::sdk::resource;
  using namespace opentelemetry::semconv;
  return Resource::Create({
      {service::kServiceName, application_name},
      {deployment::kDeploymentId, get_deployment_id()},
      {process::kProcessPid, pid},
      {host::kHostName, hostname},
  });
}

void OtelTelemetryBackend::initialize() {
  using namespace opentelemetry::trace;
  using namespace opentelemetry::sdk::trace;
  auto processor = create_span_processor(endpoint_);
  auto resource = create_resource(application_name_, hostname_, pid_);
  auto tracer_provider = TracerProviderFactory::Create(std::move(processor), resource);
  Provider::SetTracerProvider(std::move(tracer_provider));
}

OtelTelemetryBackend::~OtelTelemetryBackend() {
  using namespace opentelemetry::trace;
  // This will destroy the previously configured trace provider and flush all pending data
  Provider::SetTracerProvider(std::make_shared<NoopTracerProvider>());
}
