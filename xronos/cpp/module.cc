// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "module.hh"

PYBIND11_MODULE(_runtime, mod, py::mod_gil_not_used()) {
  mod.doc() = "Python bindings for runtime";
  define_environment(mod);
  define_reactor_element(mod);
  define_metric(mod);
  define_action(mod);
  define_port(mod);
  define_reactor(mod);
  define_reaction(mod);
  define_source_info(mod);
}
