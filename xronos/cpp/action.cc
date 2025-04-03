// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "gil_wrapper.hh"
#include "module.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/time.hh"

#include <pybind11/cast.h>
#include <pybind11/chrono.h>

using namespace xronos::runtime;

auto get_helper(Action<GilWrapper>* action) -> py::object {
  if (!action->is_present()) {
    return py::cast<py::none>(Py_None);
  }
  return action->get()->get();
}

void schedule_helper(Action<GilWrapper>* action, const py::object& value, const Duration& delay) {
  action->schedule(GilWrapper{value}, delay);
}

void define_action(py::module& mod) {
  py::class_<BaseAction, ReactorElement>(mod, "EventSource")
      .def_property_readonly("_is_present", &BaseAction::is_present);

  py::class_<Timer, BaseAction>(mod, "Timer")
      .def(py::init<std::string_view, Reactor&, const Duration&, const Duration&>())
      .def_property("_offset", &Timer::offset, &Timer::set_offset)
      .def_property("_period", &Timer::period, &Timer::set_period);

  py::class_<StartupTrigger, Timer>(mod, "Startup").def(py::init<std::string_view, Reactor&>());

  py::class_<ShutdownTrigger, Timer>(mod, "Shutdown").def(py::init<std::string_view, Reactor&>());

  py::class_<Action<GilWrapper>, BaseAction>(mod, "SchedulableEventSource")
      .def("_get", &get_helper)
      .def("_schedule", &schedule_helper);

  py::class_<LogicalAction<GilWrapper>, Action<GilWrapper>>(mod, "ProgrammableTimer")
      .def(py::init<std::string_view, Reactor&>(), py::arg("name"), py::arg("container"));

  py::class_<PhysicalAction<GilWrapper>, Action<GilWrapper>>(mod, "PhysicalEvent")
      .def(py::init<std::string_view, Reactor&>(), py::arg("name"), py::arg("container"));
}
