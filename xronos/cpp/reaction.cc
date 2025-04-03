// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "module.hh"

#include <pybind11/detail/common.h>
#include <pybind11/functional.h>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/port.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/runtime/reactor.hh"

using namespace xronos::runtime;

void define_reaction(py::module& mod) {
  py::class_<Reaction, ReactorElement>(mod, "Reaction")
      .def(py::init<std::string_view, int, Reactor&, std::function<void()>&>())
      .def("_declare_event_source_trigger", py::overload_cast<BaseAction*>(&Reaction::declare_trigger))
      .def("_declare_port_trigger", py::overload_cast<BasePort*>(&Reaction::declare_trigger))
      .def("_declare_port_effect", &Reaction::declare_antidependency)
      .def("_declare_event_source_effect", &Reaction::declare_schedulable_action)
      .def("_declare_port_source", py::overload_cast<BasePort*>(&Reaction::declare_dependency));
}
