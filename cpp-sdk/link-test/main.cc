// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// One of the translation units of the SDK link test (see CMakeLists.txt).
// It includes every public header like app.cc, its sibling in the
// executable, and runs the reactor programs from both the executable and
// the shared library.

#include "include_all.hh"

#include <cstdlib>

namespace xronos::sdk::link_test {

auto run_lib_program() -> bool;
auto run_app_program() -> bool;

} // namespace xronos::sdk::link_test

auto main() -> int {
  const bool lib_ok = xronos::sdk::link_test::run_lib_program();
  const bool app_ok = xronos::sdk::link_test::run_app_program();
  return lib_ok && app_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
