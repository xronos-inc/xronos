// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MODULE_HH
#define MODULE_HH

#include <algorithm>
#include <pybind11/pybind11.h>

namespace py = pybind11;

void define_action(py::module& mod);
void define_environment(py::module& mod);
void define_metric(py::module& mod);
void define_port(py::module& mod);
void define_reaction(py::module& mod);
void define_reactor(py::module& mod);
void define_reactor_element(py::module& mod);
void define_source_info(py::module& mod);

#endif
