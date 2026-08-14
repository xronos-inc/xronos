// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_EXCEPTIONS_HH
#define XRONOS_ABI_EXCEPTIONS_HH

#include <stdexcept>

#include "xronos/abi/contract.hh" // IWYU pragma: keep

// Exception types that cross the SDK/implementation boundary: thrown by the
// implementation, caught by name in SDK or user code. Their
// typeinfo must have default visibility in every `.so` involved, so the
// dynamic linker (or libstdc++'s name-based fallback) can match them across
// module boundaries; nothing may narrow the visibility of these types.
//
// Everything else thrown by the implementation crosses the boundary as an
// ordinary, anonymous `std::exception` (native unwinding needs no
// translation layer); only types listed here may be caught by name on the
// other side.

namespace xronos::abi::inline v1 {

// Thrown by `Backend::register_*` when the element name is malformed
// (empty, or containing whitespace or a reserved character) or already
// exists within the same parent. The message distinguishes the two cases.
// It carries no source location. The SDK decorates it before rethrowing
// to the user.
class InvalidNameError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// Thrown by `Backend::add_connection` for invalid connections and by the
// implementation's run preparation when the assembled model fails
// validation.
class ValidationError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

} // namespace xronos::abi::inline v1

#endif // XRONOS_ABI_EXCEPTIONS_HH
