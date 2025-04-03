// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "module.hh"

#include "xronos/runtime/environment.hh"
#include "xronos/runtime/reactor.hh"
#include "xronos/runtime/reactor_element.hh"

using namespace xronos::runtime;

void define_reactor_element(py::module& mod) {
  py::class_<ReactorElement>(mod, "ReactorElement")
      .def_property_readonly("_name", &ReactorElement::name)
      .def_property_readonly("_fqn", &ReactorElement::fqn)
      .def_property_readonly("_uid", &ReactorElement::uid)
      .def_property_readonly("_environment",
                             static_cast<Environment& (ReactorElement::*)()>(&ReactorElement::environment));
}
