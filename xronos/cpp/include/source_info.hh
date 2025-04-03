// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SOURCE_INFO_HH
#define SOURCE_INFO_HH

#include "xronos/messages/source_info.pb.h"
#include <pybind11/pybind11.h>

#include <pybind11/stl.h>
#include <string_view>

namespace py = pybind11;

class SourceInfo {
public:
  std::optional<std::string> class_name;
  std::string function;
  std::string file;
  std::vector<std::string> fqn;
  uint64_t uid;
  uint32_t lineno;
  uint32_t end_lineno;
  uint32_t col_offset;
  uint32_t end_col_offset;

  SourceInfo(std::optional<std::string_view> class_name, std::string_view function, std::string_view file,
             std::vector<std::string> fqn, uint64_t uid, uint32_t lineno, uint32_t end_lineno, uint32_t col_offset,
             uint32_t end_col_offset)
      : class_name(class_name)
      , function(function)
      , file(file)
      , fqn(std::move(fqn))
      , uid(uid)
      , lineno(lineno)
      , end_lineno(end_lineno)
      , col_offset(col_offset)
      , end_col_offset(end_col_offset) {}
};

void define_source_info(py::module& mod) {
  py::class_<SourceInfo>(mod, "SourceInfo")
      .def(py::init<const std::optional<std::string>, const std::string&, const std::string&,
                    const std::vector<std::string>&, uint64_t, uint32_t, uint32_t, uint32_t, uint32_t>(),
           py::arg("class_name"), py::arg("function"), py::arg("file"), py::arg("fqn"), py::arg("uid"),
           py::arg("lineno"), py::arg("end_lineno"), py::arg("col_offset"), py::arg("end_col_offset"))
      .def_readwrite("class_name", &SourceInfo::class_name)
      .def_readwrite("function", &SourceInfo::function)
      .def_readwrite("file", &SourceInfo::file)
      .def_readwrite("fqn", &SourceInfo::fqn)
      .def_readwrite("uid", &SourceInfo::uid)
      .def_readwrite("lineno", &SourceInfo::lineno)
      .def_readwrite("end_lineno", &SourceInfo::end_lineno)
      .def_readwrite("col_offset", &SourceInfo::col_offset)
      .def_readwrite("end_col_offset", &SourceInfo::end_col_offset);
}

#endif // SOURCE_INFO_HH
