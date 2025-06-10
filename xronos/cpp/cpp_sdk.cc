// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <csignal>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>

#include "pybind11/chrono.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "xronos/sdk.hh"

using namespace xronos::sdk;
namespace py = pybind11;

// Helper class that wraps arbitrary Python objects for handling in C++.
class GilWrapper {
private:
  py::object object;

public:
  // This is called from Python and we should already hold the GIL
  GilWrapper(py::object object)
      : object{std::move(object)} {}

  ~GilWrapper() {
    // This is called from C++ and we need to acquire the GIL
    py::gil_scoped_acquire acquire;
    auto ref = object.release();
    ref.dec_ref();
  }

  GilWrapper(const GilWrapper&) = delete;
  GilWrapper(GilWrapper&&) = default;
  auto operator=(const GilWrapper&) -> GilWrapper& = delete;
  auto operator=(GilWrapper&&) -> GilWrapper& = default;

  // This must be called while holding the GIL
  [[nodiscard]] auto get() const -> py::object { return object; }
};

class PyReaction : public BaseReaction {
public:
  using BaseReaction::BaseReaction;
  using BaseReaction::context;
  using BaseReaction::MetricEffect;
  using BaseReaction::PortEffect;
  using BaseReaction::ProgrammableTimerEffect;
  using BaseReaction::Trigger;

  void set_handler(std::function<void()> handler) noexcept {
    assert(handler_ == nullptr);
    handler_ = std::move(handler);
  }

private:
  void handler() final {
    assert(handler_ != nullptr);
    handler_();
  }

  std::function<void()> handler_{nullptr};
};

class PyEnvironment : public Environment {
public:
  class SigintHandler {
  private:
    constexpr static auto SIGINT_POLL_INTERVAL = std::chrono::milliseconds(100);
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
        environment->request_shutdown();
        std::cout << "SIGINT received. Requesting shutdown of the environment (press "
                     "Ctrl+C again to force)\n";
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

  PyEnvironment(unsigned workers, bool fast, bool render_reactor_graph, Duration timeout = Duration::max())
      : Environment(workers, fast, timeout, render_reactor_graph) {}

  void execute() {
    py::gil_scoped_release release;
    SigintHandler sigint_handler(this); // acts as a scoped guard
    Environment::execute();
  }
};

class PyReactor : public Reactor {
public:
  PyReactor(std::string_view name, Context parent_context, std::function<void()> assemble_callback)
      : Reactor{name, parent_context}
      , assemble_callback_{std::move(assemble_callback)} {}

  using Reactor::get_lag;
  using Reactor::get_time;
  using Reactor::get_time_since_startup;
  using Reactor::request_shutdown;
  using Reactor::shutdown;
  using Reactor::startup;

  static auto add_reaction(Reactor& reactor, std::string_view name,
                           detail::SourceLocationView location) -> const PyReaction& {
    return detail::add_reaction<PyReaction>(reactor, name, location);
  }

  void connect(const InputPort<GilWrapper>& from, const InputPort<GilWrapper>& to) { Reactor::connect(from, to); }
  void connect(const OutputPort<GilWrapper>& from, const InputPort<GilWrapper>& to) { Reactor::connect(from, to); }
  void connect(const OutputPort<GilWrapper>& from, const OutputPort<GilWrapper>& to) { Reactor::connect(from, to); }
  void connect(const InputPort<GilWrapper>& from, const InputPort<GilWrapper>& to, Duration delay) {
    Reactor::connect(from, to, delay);
  }
  void connect(const OutputPort<GilWrapper>& from, const InputPort<GilWrapper>& to, Duration delay) {
    Reactor::connect(from, to, delay);
  }
  void connect(const OutputPort<GilWrapper>& from, const OutputPort<GilWrapper>& to, Duration delay) {
    Reactor::connect(from, to, delay);
  }

private:
  void assemble() final { assemble_callback_(); }

  std::function<void()> assemble_callback_;
};

PYBIND11_MODULE(_cpp_sdk, mod, py::mod_gil_not_used()) {
  mod.doc() = "Python bindings for the Xronos C++ SDK";

  auto validation_error = py::register_exception<ValidationError>(mod, "ValidationError");
  validation_error.doc() = "Exception that is thrown when a program reaches an invalid state.";

  py::class_<detail::SourceLocationView>(mod, "SourceLocation")
      .def(py::init<std::string_view, std::string_view, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>(),
           py::arg("file_"), py::arg("function"), py::arg("start_line"), py::arg("end_line"), py::arg("start_column"),
           py::arg("end_column"));

  py::class_<EnvironmentContext>(mod, "EnvironmentContext")
      .def(py::init(py::overload_cast<Environment&, detail::SourceLocationView>(&detail::create_context)),
           py::arg("environment"), py::arg("source_location"));

  py::class_<ReactorContext>(mod, "ReactorContext")
      .def(py::init(py::overload_cast<Reactor&, detail::SourceLocationView>(&detail::create_context)),
           py::arg("reactor"), py::arg("source_location"));

  py::class_<Environment>(mod, "BaseEnvironment");

  py::class_<PyEnvironment, Environment>(mod, "Environment")
      .def(py::init<unsigned, bool, bool>(), py::arg("workers"), py::arg("fast"), py::arg("render_reactor_graph"))
      .def(py::init<unsigned, bool, bool, Duration>(), py::arg("workers"), py::arg("fast"),
           py::arg("render_reactor_graph"), py::arg("timeout"))
      .def("execute", &PyEnvironment::execute)
      .def("request_shutdown", &PyEnvironment::request_shutdown)
      .def("enable_telemetry", &PyEnvironment::enable_telemetry, py::arg("application_name"), py::arg("endpoint"))
      .def("connect",
           py::overload_cast<const InputPort<GilWrapper>&, const InputPort<GilWrapper>&>(
               &PyEnvironment::connect<GilWrapper>),
           py::arg("from_"), py::arg("to"))
      .def("connect",
           py::overload_cast<const OutputPort<GilWrapper>&, const OutputPort<GilWrapper>&>(
               &PyEnvironment::connect<GilWrapper>),
           py::arg("from_"), py::arg("to"))
      .def("connect",
           py::overload_cast<const OutputPort<GilWrapper>&, const InputPort<GilWrapper>&>(
               &PyEnvironment::connect<GilWrapper>),
           py::arg("from_"), py::arg("to"))
      .def("connect_delayed",
           py::overload_cast<const InputPort<GilWrapper>&, const InputPort<GilWrapper>&, Duration>(
               &PyEnvironment::connect<GilWrapper>),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("connect_delayed",
           py::overload_cast<const OutputPort<GilWrapper>&, const OutputPort<GilWrapper>&, Duration>(
               &PyEnvironment::connect<GilWrapper>),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("connect_delayed",
           py::overload_cast<const OutputPort<GilWrapper>&, const InputPort<GilWrapper>&, Duration>(
               &PyEnvironment::connect<GilWrapper>),
           py::arg("from_"), py::arg("to"), py::arg("delay"));

  py::class_<Element>(mod, "Element")
      .def_property_readonly("name", &Element::name)
      .def_property_readonly("fqn", &Element::fqn)
      .def("add_attribute", &Element::add_attribute)
      .def("add_attributes", &Element::add_attributes<std::unordered_map<std::string, AttributeValue>>);

  py::class_<Reactor, Element>(mod, "BaseReactor");

  py::class_<PyReactor, Reactor>(mod, "Reactor")
      .def(py::init<std::string_view, EnvironmentContext, std::function<void()>>(), py::arg("name"), py::arg("context"),
           py::arg("assemble_callback"))
      .def(py::init<std::string_view, ReactorContext, std::function<void()>>(), py::arg("name"), py::arg("context"),
           py::arg("assemble_callback"))
      .def("get_lag", &PyReactor::get_lag)
      .def("get_time", &PyReactor::get_time)
      .def("get_time_since_startup", &PyReactor::get_time_since_startup)
      .def("request_shutdown", &PyReactor::request_shutdown)
      .def_property_readonly("startup", &PyReactor::startup)
      .def_property_readonly("shutdown", &PyReactor::shutdown)
      .def("add_reaction", &PyReactor::add_reaction, py::return_value_policy::reference, py::arg("name"),
           py::arg("source_location"))
      .def("connect",
           py::overload_cast<const InputPort<GilWrapper>&, const InputPort<GilWrapper>&>(&PyReactor::connect),
           py::arg("from_"), py::arg("to"))
      .def("connect",
           py::overload_cast<const OutputPort<GilWrapper>&, const OutputPort<GilWrapper>&>(&PyReactor::connect),
           py::arg("from_"), py::arg("to"))
      .def("connect",
           py::overload_cast<const OutputPort<GilWrapper>&, const InputPort<GilWrapper>&>(&PyReactor::connect),
           py::arg("from_"), py::arg("to"))
      .def("connect_delayed",
           py::overload_cast<const InputPort<GilWrapper>&, const InputPort<GilWrapper>&, Duration>(&PyReactor::connect),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("connect_delayed",
           py::overload_cast<const OutputPort<GilWrapper>&, const OutputPort<GilWrapper>&, Duration>(
               &PyReactor::connect),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def(
          "connect_delayed",
          py::overload_cast<const OutputPort<GilWrapper>&, const InputPort<GilWrapper>&, Duration>(&PyReactor::connect),
          py::arg("from_"), py::arg("to"), py::arg("delay"));

  py::class_<EventSource<GilWrapper>, Element>(mod, "EventSource");

  py::class_<EventSource<void>, Element>(mod, "VoidEventSource");

  py::class_<Startup, EventSource<void>>(mod, "Startup");

  py::class_<Shutdown, EventSource<void>>(mod, "Shutdown");

  py::class_<PeriodicTimer, EventSource<void>>(mod, "PeriodicTimer")
      .def(py::init<std::string_view, ReactorContext, Duration, Duration>(), py::arg("name"), py::arg("context"),
           py::arg("period"), py::arg("offset"))
      .def("offset", &PeriodicTimer::offset)
      .def("period", &PeriodicTimer::period)
      .def("set_offset", &detail::set_timer_offset, py::arg("offset"))
      .def("set_period", &detail::set_timer_period, py::arg("period"));

  py::class_<Port<GilWrapper>, EventSource<GilWrapper>>(mod, "Port");

  py::class_<InputPort<GilWrapper>, Port<GilWrapper>>(mod, "InputPort")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"));

  py::class_<OutputPort<GilWrapper>, Port<GilWrapper>>(mod, "OutputPort")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"));

  py::class_<ProgrammableTimer<GilWrapper>, EventSource<GilWrapper>>(mod, "ProgrammableTimer")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"));

  py::class_<PhysicalEvent<GilWrapper>, EventSource<GilWrapper>>(mod, "PhysicalEvent")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"))
      .def(
          "trigger",
          [](PhysicalEvent<GilWrapper>& event, const py::object& value) { event.trigger(GilWrapper{value}); },
          py::arg("value"));

  py::class_<Metric, Element>(mod, "Metric")
      .def(py::init<std::string_view, ReactorContext, std::string_view, std::string_view>(), py::arg("name"),
           py::arg("context"), py::arg("description"), py::arg("unit"))
      .def_property_readonly("description", &Metric::description)
      .def_property_readonly("unit", &Metric::unit);

  py::class_<ReactionContext>(mod, "ReactionContext");

  py::class_<BaseReaction, Element>(mod, "BaseReaction");

  py::class_<PyReaction, BaseReaction>(mod, "Reaction")
      .def("set_handler", &PyReaction::set_handler, py::arg("handler"))
      .def("context", &PyReaction::context);

  py::class_<PyReaction::Trigger<void>>(mod, "VoidTrigger")
      .def(py::init<const EventSource<void>&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def("is_present", &PyReaction::Trigger<void>::is_present);

  py::class_<PyReaction::Trigger<GilWrapper>>(mod, "Trigger")
      .def(py::init<const EventSource<GilWrapper>&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def("is_present", &PyReaction::Trigger<GilWrapper>::is_present)
      .def("get", [](const PyReaction::Trigger<GilWrapper>& trigger) { return trigger.get()->get(); });

  py::class_<PyReaction::PortEffect<GilWrapper>>(mod, "PortEffect")
      .def(py::init<Port<GilWrapper>&, ReactionContext>(), py::arg("port"), py::arg("context"))
      .def(
          "set",
          [](PyReaction::PortEffect<GilWrapper>& effect, const py::object& value) { effect.set(GilWrapper{value}); },
          py::arg("value"))
      .def("is_present", &PyReaction::PortEffect<GilWrapper>::is_present)
      .def("get", [](const PyReaction::PortEffect<GilWrapper>& effect) { return effect.get()->get(); });

  py::class_<PyReaction::ProgrammableTimerEffect<GilWrapper>>(mod, "ProgrammableTimerEffect")
      .def(py::init<ProgrammableTimer<GilWrapper>&, ReactionContext>(), py::arg("programmable_timer"),
           py::arg("context"))
      .def(
          "schedule",
          [](PyReaction::ProgrammableTimerEffect<GilWrapper>& effect, const py::object& value, Duration delay) {
            effect.schedule(GilWrapper{value}, delay);
          },
          py::arg("value"), py::arg("delay"));

  py::class_<PyReaction::MetricEffect>(mod, "MetricEffect")
      .def(py::init<Metric&, ReactionContext>(), py::arg("metric"), py::arg("context"))
      .def("record", py::overload_cast<std::int64_t>(&PyReaction::MetricEffect::record), py::arg("value"))
      .def("record", py::overload_cast<double>(&PyReaction::MetricEffect::record), py::arg("value"));
}
