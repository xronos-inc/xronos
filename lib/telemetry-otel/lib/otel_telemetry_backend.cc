// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/otel/otel_telemetry_backend.hh"

#include "opentelemetry/context/context.h"
#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_factory.h"
#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_options.h"
#include "opentelemetry/sdk/resource/semantic_conventions.h"
#include "opentelemetry/sdk/trace/batch_span_processor_factory.h"
#include "opentelemetry/sdk/trace/batch_span_processor_options.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
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

auto create_resource(std::string_view application_name, std::string_view hostname, std::int64_t pid) {
  using namespace opentelemetry::sdk::resource;
  using namespace opentelemetry::sdk::resource::SemanticConventions;
  return Resource::Create({
      {kServiceName, application_name},
      {kProcessPid, pid},
      {kHostName, hostname},
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

void OtelTelemetryBackend::shutdown() {
  using namespace opentelemetry::trace;
  // This will destroy the previously configured trace provider and flush all pending data
  Provider::SetTracerProvider(std::make_shared<NoopTracerProvider>());
}
