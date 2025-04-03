// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <pybind11/stl.h>

#include "module.hh"

#include "xronos/telemetry/metric.hh"

using namespace xronos::runtime;
using namespace xronos::telemetry;

void define_metric(py::module& mod) {
  py::class_<MetricDataLogger>(mod, "MetricDataLogger");

  py::class_<MetricDataLoggerProvider>(mod, "MetricDataLoggerProvider")
      .def(py::init<>())
      .def("get_logger", &MetricDataLoggerProvider::logger)
      .def("set_logger", &MetricDataLoggerProvider::set_logger)
      .def("reset_logger", &MetricDataLoggerProvider::reset_logger);

  py::class_<Metric, ReactorElement>(mod, "Metric")
      .def(py::init<std::string_view, Reactor&, MetricDataLoggerProvider&, std::string_view, std::string_view>())
      .def_property_readonly("_description", &Metric::description)
      .def_property_readonly("_unit", &Metric::unit)
      .def("_record", &Metric::record);
}
