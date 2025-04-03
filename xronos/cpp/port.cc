// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "gil_wrapper.hh"
#include "module.hh"

#include "xronos/runtime/connection.hh"
#include "xronos/runtime/port.hh"

using namespace xronos::runtime;

auto get_helper(Port<GilWrapper>* port) -> py::object { return port->get()->get(); }

void set_helper(Port<GilWrapper>* port, const py::object& value) { port->set(GilWrapper{value}); }

void define_port(py::module& mod) {
  py::class_<BasePort, ReactorElement>(mod, "BasePort");
  py::class_<Port<GilWrapper>, BasePort>(mod, "Port")
      .def("_set", &set_helper)
      .def("_get", &get_helper)
      .def_property_readonly("_is_present", &BasePort::is_present);
  py::class_<Input<GilWrapper>, Port<GilWrapper>>(mod, "Input").def(py::init<std::string_view, Reactor&>());
  py::class_<Output<GilWrapper>, Port<GilWrapper>>(mod, "Output").def(py::init<std::string_view, Reactor&>());
}
