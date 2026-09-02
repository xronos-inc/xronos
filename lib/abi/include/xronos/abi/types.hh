// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_TYPES_HH
#define XRONOS_ABI_TYPES_HH

#include <chrono>
#include <cstdint>
#include <string>

#include "xronos/abi/contract.hh" // IWYU pragma: keep

// Frozen value types of the SDK/implementation ABI. These are plain data
// types whose layout is part of the ABI contract: they may never change
// within a major ABI version.

namespace xronos::abi::inline v1 {

// Identifies an element (reactor, port, timer, reaction, ...) across the
// boundary. Uids are assigned by the implementation, returned from the
// `Backend::register_*` methods, and are the only element identity the SDK
// holds.
using ElementUid = std::uint64_t;

using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

// Source location of an element's declaration in user code.
struct SourceLocation {
  std::string file;
  std::string function;
  std::uint32_t start_line{0};
  std::uint32_t end_line{0};
  std::uint32_t start_column{0};
  std::uint32_t end_column{0};
};

// Outcome of an external trigger attempt. The enumerator values and the
// underlying type are frozen within a major ABI version. A minor version
// may append enumerators; a consumer treats an unrecognized status as a
// dropped event.
enum class TriggerStatus : std::uint8_t {
  Accepted = 0,
  NotStarted = 1,
  Stopped = 2,
  // The uid resolves to no physical event of the running program.
  UnknownPhysicalEvent = 3,
};

} // namespace xronos::abi::inline v1

#endif // XRONOS_ABI_TYPES_HH
