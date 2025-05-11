// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "gil_wrapper.hh"
#include "module.hh"
#include "source_info.hh"

#include "xronos/graph_exporter/exporter.hh"
#include "xronos/messages/source_info.pb.h"
#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/port.hh"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/otel/otel_telemetry_backend.hh"
#include "xronos/telemetry/telemetry.hh"

#include <csignal>
#include <memory>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <string_view>

using namespace xronos::graph_exporter;
using namespace xronos::runtime;
using namespace xronos::telemetry;

// 100ms gives the illusion of responsiveness to an interrupt signal
// because it is roughly the limit at which humans can perceive a delay
constexpr auto SIGINT_POLL_INTERVAL = std::chrono::milliseconds(100);

void export_source_info(const std::vector<SourceInfo>& source_infos_vector,
                        xronos::messages::source_info::SourceInfo& source_infos) {
  for (const auto& source_info : source_infos_vector) {
    auto* proto_source_info = source_infos.add_infos();
    auto* source_location = proto_source_info->mutable_frame();
    source_location->set_file(source_info.file);
    source_location->set_lineno(source_info.lineno);
    source_location->set_col_offset(source_info.col_offset);
    source_location->set_function(source_info.function);
    source_location->set_end_lineno(source_info.end_lineno);
    source_location->set_end_col_offset(source_info.end_col_offset);
    for (const auto& fqn : source_info.fqn) {
      proto_source_info->add_fqn(fqn);
    }
    proto_source_info->set_uid(source_info.uid);
    if (source_info.class_name.has_value()) {
      proto_source_info->set_class_name(source_info.class_name.value());
    }
  }
}

void send_program_info_assembled(Environment* environment, const std::vector<SourceInfo>& source_infos,
                                 const xronos::telemetry::AttributeManager& attribute_manager) {
  xronos::messages::source_info::SourceInfo source_info;
  export_source_info(source_infos, source_info);
  send_reactor_graph_to_diagram_server(*environment, source_info, attribute_manager);
}

class SigintHandler {
private:
  inline static std::atomic<bool> sigint_called{false};
  static void siginit_handler([[maybe_unused]] int signal) { sigint_called = true; }
  std::mutex mtx_exited;
  bool exited{false};
  std::condition_variable cv_exited;

  void (*external_sigint_handler)(int){nullptr};

  std::thread polling_thread;

  void sigint_polling(Environment* environment) {
    // Poll because it is not generally safe to access condition variables from signal
    // handlers
    std::unique_lock lock(mtx_exited);
    // This thread checks periodically if SIGINT has been received. As an optimization,
    // it exits early without waiting for the next polling interval if the environment
    // has already exited.
    while (!sigint_called && !exited) {
      cv_exited.wait_for(lock, SIGINT_POLL_INTERVAL);
    }
    if (sigint_called) {
      environment->async_shutdown();
      log::Info() << "SIGINT received. Requesting shutdown of the environment (press "
                     "Ctrl+C again to force)";
      (void)signal(SIGINT, SIG_DFL); // Restore the default signal handler to allow the
                                     // user to force the shutdown by pressing Ctrl+C again
    }
  }

public:
  SigintHandler(Environment* environment)
      : external_sigint_handler(signal(SIGINT, &siginit_handler)) {
    polling_thread = std::thread([this, environment]() { sigint_polling(environment); });
  }
  ~SigintHandler() {
    {
      std::unique_lock lock(mtx_exited);
      exited = true;
      cv_exited.notify_all();
    }
    polling_thread.join();
    if (sigint_called) {
      external_sigint_handler(SIGINT); // run CPython's default signal handler, which will
                                       // raise a KeyboardInterrupt or trigger any
                                       // user-defined Python-level signal handler at the
                                       // next Python bytecode instruction
    }
    (void)signal(SIGINT, external_sigint_handler);
  }
  SigintHandler(const SigintHandler&) = delete;
  SigintHandler(SigintHandler&&) = delete;
  void operator=(const SigintHandler&) = delete;
  void operator=(SigintHandler&&) = delete;
};

class PyEnvironment : public Environment {
private:
  inline static std::atomic<bool> environment_executing{false};

  AttributeManager attribute_manager_;
  MetricDataLoggerProvider metric_data_logger_provider_;

  std::shared_ptr<TelemetryBackend> telemetry_backend_;

public:
  PyEnvironment(unsigned int num_workers, bool fast_fwd_execution, const Duration& timeout = Duration::max())
      : Environment(num_workers, fast_fwd_execution, timeout)
      , telemetry_backend_(std::make_shared<NoopTelemetryBackend>()) {}

  auto attribute_manager() -> auto& { return attribute_manager_; }
  auto metric_data_logger_provider() -> auto& { return metric_data_logger_provider_; }

  auto add_attribute(const ReactorElement& element, std::string_view key, const AttributeValue& value) -> bool {
    return attribute_manager_.add_attribute(element, key, value);
  }

  auto add_attributes(const ReactorElement& element, AttributeMap attributes) -> bool {
    return attribute_manager_.add_attributes(element, std::move(attributes));
  }

  auto set_telemetry_backend(std::shared_ptr<TelemetryBackend> backend) {
    set_data_logger(backend->runtime_data_logger());
    metric_data_logger_provider_.set_logger(backend->metric_data_logger());
    telemetry_backend_ = std::move(backend);
  }

  void execute(const std::vector<SourceInfo>& source_infos) {
    if (environment_executing.exchange(true)) {
      throw std::runtime_error("an Environment is already executing; invocation of execute() is not reentrant");
    }
    py::gil_scoped_release release;

    SigintHandler sigint_handler(this); // acts as a scoped guard

    std::optional<std::thread> startup_thread;
    try {
      telemetry_backend_->initialize();
      assemble();
      send_program_info_assembled(this, source_infos, attribute_manager_);
      startup_thread = startup();
    } catch (const std::exception& e) {
      set_exception(); // exception will be re-thrown after cleanup
    }
    if (startup_thread.has_value()) {
      startup_thread->join();
    }
    telemetry_backend_->shutdown();
    environment_executing = false;
    rethrow_exception_if_any();
  }

  void connect(Port<GilWrapper>& from, Port<GilWrapper>& to) { draw_connection(from, to, ConnectionProperties{}); }

  void connect_delayed(Port<GilWrapper>& from, Port<GilWrapper>& to, const Duration& delay) {
    draw_connection(from, to, ConnectionProperties{ConnectionType::Delayed, delay});
  }
};

void define_environment(py::module& mod) {
  py::class_<AttributeManager>(mod, "AttributeManager");
  py::class_<TelemetryBackend, std::shared_ptr<TelemetryBackend>>(mod, "TelemetryBackend");
  py::class_<otel::OtelTelemetryBackend, TelemetryBackend, std::shared_ptr<otel::OtelTelemetryBackend>>(
      mod, "OtelTelemetryBackend")
      .def(py::init<AttributeManager&, std::string_view, std::string_view, std::string_view, std::int64_t>(),
           py::arg("attribute_manager"), py::arg("application_name"), py::arg("endpoint"), py::arg("hostname"),
           py::arg("pid"));

  py::class_<Environment>(mod, "BaseEnvironment");
  py::class_<PyEnvironment, Environment>(mod, "Environment")
      .def(py::init<unsigned int, bool>(), py::arg("workers"), py::arg("fast"))
      .def(py::init<unsigned int, bool, const Duration&>(), py::arg("workers"), py::arg("fast"), py::arg("timeout"))
      .def("_execute", &PyEnvironment::execute)
      .def("_connect", &PyEnvironment::connect)
      .def("_connect_delayed", &PyEnvironment::connect_delayed)
      .def("request_shutdown", &Environment::async_shutdown)
      .def_property_readonly("_top_level_reactors", &Environment::top_level_reactors)
      .def_property_readonly("_metric_data_logger_provider", &PyEnvironment::metric_data_logger_provider)
      .def_property_readonly("_attribute_manager", &PyEnvironment::attribute_manager)
      .def("_add_attribute", &PyEnvironment::add_attribute)
      .def("_add_attributes", &PyEnvironment::add_attributes)
      .def("_set_telemetry_backend", &PyEnvironment::set_telemetry_backend);
  py::register_exception<ValidationError>(mod, "ValidationError");
}
