// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cassert>
#include <csignal>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "pybind11/cast.h"
#include "pybind11/chrono.h"     // IWYU pragma: keep
#include "pybind11/functional.h" // IWYU pragma: keep
#include "pybind11/gil.h"
#include "pybind11/pybind11.h"
#include "pybind11/pytypes.h"
#include "pybind11/stl.h" // IWYU pragma: keep

#include "xronos/sdk.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/periodic_timer.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/shutdown.hh"
#include "xronos/sdk/startup.hh"
#include "xronos/sdk/time.hh"

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
  using BaseReaction::current_time;
  using BaseReaction::deadline;
  using BaseReaction::elapsed_time;
  using BaseReaction::is_before_deadline;
  using BaseReaction::lag;
  using BaseReaction::MetricEffect;
  using BaseReaction::PortEffect;
  using BaseReaction::ProgrammableTimerEffect;
  using BaseReaction::ShutdownEffect;
  using BaseReaction::slack;
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
  // Replaces Python's signal handler with the default handler and restores
  // Python's handler when destructed. This ensures that Ctrl+C aborts the
  // program immediately, but it does not raise a KeyboardInterrupt while a
  // Xronos program is running.
  class SigintHandler {
  public:
    SigintHandler()
        : python_sigint_handler(signal(SIGINT, SIG_DFL)) {}
    ~SigintHandler() {
      // Restore the Python handler
      (void)signal(SIGINT, python_sigint_handler);
    }
    SigintHandler(const SigintHandler&) = delete;
    SigintHandler(SigintHandler&&) = delete;
    void operator=(const SigintHandler&) = delete;
    void operator=(SigintHandler&&) = delete;

  private:
    void (*python_sigint_handler)(int){nullptr};
  };

  PyEnvironment(unsigned workers, bool fast, bool render_reactor_graph, Duration timeout = Duration::max())
      : Environment(workers, fast, timeout, render_reactor_graph) {}

  void execute() {
    py::gil_scoped_release release;
    SigintHandler sigint_handler{}; // acts as a scoped guard
    Environment::execute();
  }
};

class PyReactor : public Reactor {
public:
  PyReactor(std::string_view name, Context parent_context, std::function<void()> assemble_callback)
      : Reactor{name, parent_context}
      , assemble_callback_{std::move(assemble_callback)} {}

  using Reactor::context;
  // The reactor-level timing API is deprecated in favor of the reaction-scoped
  // API (ADR 0049). The Python SDK still exposes the reactor-level methods for
  // backward compatibility, so we keep exposing them and silence the deprecation warning locally.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  using Reactor::get_lag;
  using Reactor::get_time;
  using Reactor::get_time_since_startup;
#pragma GCC diagnostic pop
  using Reactor::shutdown;
  using Reactor::startup;

  static auto add_reaction(Reactor& reactor, std::string_view name, std::optional<Duration> deadline,
                           detail::SourceLocationView location) -> const PyReaction& {
    return detail::add_reaction<PyReaction>(reactor, name, deadline, location);
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
  // Overwriting the module ensures that Python tracebacks show
  // `xronos.ValidationError` instead of `xronos._cpp_sdk.ValidationError` when
  // the exception is thrown.
  validation_error.attr("__module__") = "xronos";

  auto duplicate_name_error = py::register_exception<DuplicateNameError>(mod, "DuplicateNameError");
  duplicate_name_error.doc() = "Exception that is thrown when creating an element with a name that is already in use.";
  duplicate_name_error.attr("__module__") = "xronos";

  py::class_<detail::SourceLocationView>(mod, "SourceLocation")
      .def(py::init<std::string_view, std::string_view, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>(),
           py::arg("file_"), py::arg("function"), py::arg("start_line"), py::arg("end_line"), py::arg("start_column"),
           py::arg("end_column"))
      .def_readonly("file_", &detail::SourceLocationView::file)
      .def_readonly("function", &detail::SourceLocationView::function)
      .def_readonly("start_line", &detail::SourceLocationView::start_line)
      .def_readonly("end_line", &detail::SourceLocationView::end_line)
      .def_readonly("start_column", &detail::SourceLocationView::start_column)
      .def_readonly("end_column", &detail::SourceLocationView::end_column);

  py::class_<EnvironmentContext>(mod, "EnvironmentContext");
  py::class_<ReactorContext>(mod, "ReactorContext");

  py::class_<Environment>(mod, "BaseEnvironment");

  py::class_<PyEnvironment, Environment>(mod, "Environment")
      .def(py::init<unsigned, bool, bool>(), py::arg("workers"), py::arg("fast"), py::arg("render_reactor_graph"))
      .def(py::init<unsigned, bool, bool, Duration>(), py::arg("workers"), py::arg("fast"),
           py::arg("render_reactor_graph"), py::arg("timeout"))
      .def("execute", &PyEnvironment::execute)
      .def("enable_telemetry", &PyEnvironment::enable_telemetry, py::arg("application_name"), py::arg("endpoint"))
      .def("connect",
           static_cast<void (Environment::*)(const InputPort<GilWrapper>&, const InputPort<GilWrapper>&)>(
               &Environment::connect),
           py::arg("from_"), py::arg("to"))
      .def("connect",
           static_cast<void (Environment::*)(const OutputPort<GilWrapper>&, const OutputPort<GilWrapper>&)>(
               &Environment::connect),
           py::arg("from_"), py::arg("to"))
      .def("connect",
           static_cast<void (Environment::*)(const OutputPort<GilWrapper>&, const InputPort<GilWrapper>&)>(
               &Environment::connect),
           py::arg("from_"), py::arg("to"))
      .def("connect_delayed",
           static_cast<void (Environment::*)(const InputPort<GilWrapper>&, const InputPort<GilWrapper>&, Duration)>(
               &Environment::connect),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("connect_delayed",
           static_cast<void (Environment::*)(const OutputPort<GilWrapper>&, const OutputPort<GilWrapper>&, Duration)>(
               &Environment::connect),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("connect_delayed",
           static_cast<void (Environment::*)(const OutputPort<GilWrapper>&, const InputPort<GilWrapper>&, Duration)>(
               &Environment::connect),
           py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("context", py::overload_cast<detail::SourceLocationView>(&PyEnvironment::context),
           py::arg("source_location"));

  py::class_<Element>(mod, "Element")
      .def_property_readonly("name", &Element::name)
      .def_property_readonly("fqn", &Element::fqn)
      .def("add_attribute", &Element::add_attribute)
      .def("add_attributes", &Element::add_attributes<std::unordered_map<std::string, AttributeValue>>);

  py::class_<Reactor, Element>(mod, "BaseReactor");

  auto reactor_class = py::class_<PyReactor, Reactor>(mod, "Reactor")
                           .def(py::init<std::string_view, EnvironmentContext, std::function<void()>>(),
                                py::arg("name"), py::arg("context"), py::arg("assemble_callback"))
                           .def(py::init<std::string_view, ReactorContext, std::function<void()>>(), py::arg("name"),
                                py::arg("context"), py::arg("assemble_callback"));

  // The reactor-level timing API is deprecated (ADR 0049); the Python API still
  // exposes it until it is migrated. Register these in a separate statement so
  // the deprecation warning can be silenced locally (a #pragma is not allowed
  // inside the chained .def(...) expression above).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  reactor_class.def("get_lag", &PyReactor::get_lag)
      .def("get_time", &PyReactor::get_time)
      .def("get_time_since_startup", &PyReactor::get_time_since_startup);
#pragma GCC diagnostic pop

  reactor_class.def_property_readonly("startup", &PyReactor::startup)
      .def_property_readonly("shutdown", &PyReactor::shutdown)
      .def("add_reaction", &PyReactor::add_reaction, py::return_value_policy::reference, py::arg("name"),
           py::arg("deadline"), py::arg("source_location"))
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
          py::arg("from_"), py::arg("to"), py::arg("delay"))
      .def("context", py::overload_cast<detail::SourceLocationView>(&PyReactor::context), py::arg("source_location"));

  py::class_<Startup, Element>(mod, "Startup");

  py::class_<Shutdown, Element>(mod, "Shutdown");

  py::class_<PeriodicTimer, Element>(mod, "PeriodicTimer")
      .def(py::init<std::string_view, ReactorContext, Duration, Duration>(), py::arg("name"), py::arg("context"),
           py::arg("period"), py::arg("offset"))
      .def("offset", &PeriodicTimer::offset)
      .def("period", &PeriodicTimer::period)
      .def("set_offset", &detail::set_timer_offset, py::arg("offset"))
      .def("set_period", &detail::set_timer_period, py::arg("period"));

  py::class_<InputPort<GilWrapper>, Element>(mod, "InputPort")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"));

  py::class_<OutputPort<GilWrapper>, Element>(mod, "OutputPort")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"));

  py::class_<ProgrammableTimer<GilWrapper>, Element>(mod, "ProgrammableTimer")
      .def(py::init<std::string_view, ReactorContext>(), py::arg("name"), py::arg("context"));

  py::class_<PhysicalEvent<GilWrapper>, Element>(mod, "PhysicalEvent")
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
      .def("context", &PyReaction::context)
      .def("lag", &PyReaction::lag)
      .def("current_time", &PyReaction::current_time)
      .def("elapsed_time", &PyReaction::elapsed_time)
      .def("deadline", &PyReaction::deadline)
      .def("slack",
           [](const PyReaction& reaction) -> py::object {
             auto slack = reaction.slack();
             // When there is no deadline, the SDK reports Duration::max() as a
             // sentinel for "unbounded". Translate this to the idiomatic Python
             // sentinel datetime.timedelta.max. The comparison is exact since
             // this is the only case that yields Duration::max().
             if (slack == Duration::max()) {
               return py::module_::import("datetime").attr("timedelta").attr("max");
             }
             return py::cast(slack);
           })
      .def("is_before_deadline", &PyReaction::is_before_deadline);

  py::class_<PyReaction::Trigger<void>>(mod, "VoidTrigger")
      .def(py::init<const PeriodicTimer&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def(py::init<const Startup&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def(py::init<const Shutdown&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def("is_present", &PyReaction::Trigger<void>::is_present);

  py::class_<PyReaction::Trigger<GilWrapper>>(mod, "Trigger")
      .def(py::init<const InputPort<GilWrapper>&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def(py::init<const OutputPort<GilWrapper>&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def(py::init<const PhysicalEvent<GilWrapper>&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def(py::init<const ProgrammableTimer<GilWrapper>&, ReactionContext>(), py::arg("trigger"), py::arg("context"))
      .def("is_present", &PyReaction::Trigger<GilWrapper>::is_present)
      .def("get", [](const PyReaction::Trigger<GilWrapper>& trigger) { return trigger.get()->get(); });

  py::class_<PyReaction::PortEffect<GilWrapper>>(mod, "PortEffect")
      .def(py::init<InputPort<GilWrapper>&, ReactionContext>(), py::arg("effect"), py::arg("context"))
      .def(py::init<OutputPort<GilWrapper>&, ReactionContext>(), py::arg("effect"), py::arg("context"))
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

  py::class_<PyReaction::ShutdownEffect>(mod, "ShutdownEffect")
      .def(py::init<Shutdown&, ReactionContext>(), py::arg("shutdown"), py::arg("context"))
      .def("trigger_shutdown", &PyReaction::ShutdownEffect::trigger_shutdown);
}
