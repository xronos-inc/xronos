// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef GIL_WRAPPER_HH
#define GIL_WRAPPER_HH

#include <algorithm>
#include <pybind11/pybind11.h>

namespace py = pybind11;

class __attribute__((visibility("hidden"))) GilWrapper {
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

#endif // GIL_WRAPPER_HH
