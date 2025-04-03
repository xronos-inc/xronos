// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "module.hh"

#include "xronos/runtime/reactor.hh"
#include "xronos/runtime/reactor_element.hh"

#include <pybind11/chrono.h>
#include <pybind11/stl.h>

using namespace xronos::runtime;

class AssembleTrampoline : public Reactor {
public:
  using Reactor::Reactor;
  void assemble() override { PYBIND11_OVERRIDE_PURE_NAME(void, Reactor, "_assemble", assemble); }
};

void define_reactor(py::module& mod) {
  py::class_<Reactor, AssembleTrampoline, ReactorElement>(mod, "Reactor")
      .def(py::init<std::string_view, Environment&>())
      .def(py::init<std::string_view, Reactor&>())
      .def("_assemble", &Reactor::assemble)
      .def_static("_get_physical_time", &Reactor::get_physical_time)
      .def("_get_logical_time", &Reactor::get_logical_time)
      .def("_get_elapsed_logical_time", &Reactor::get_elapsed_logical_time)
      .def_property_readonly("_reactor_instances", &Reactor::contained_reactors);
}
