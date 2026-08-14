// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_VERSION_HH
#define XRONOS_ABI_VERSION_HH

#include <cstdint>

// Deliberately included for its compile-time toolchain checks: version.hh is
// the header a minimal handshake consumer includes, and any translation unit
// that touches the ABI must satisfy the contract.
#include "xronos/abi/contract.hh" // IWYU pragma: keep

// The version of the ABI between the header-only SDK and its implementation.
//
// This pair is the only compatibility gate between an SDK-compiled artifact
// (a node `.so` or an app with a `main()`) and an implementation `.so`.
// Package versions (SDK, CLI, implementation releases) are diagnostics only.
//
// Rules:
//  - Appending a virtual method to the end of an ABI interface, or a field
//    to the end of `NodeDescriptor` (node.hh), bumps the minor version.
//  - Any other change to the ABI surface (reordering, insertion, removal,
//    changing a signature or a frozen value type) bumps the major version.
//  - An implementation accepts a consumer iff the majors are equal and the
//    implementation's minor is >= the consumer's minor (the `supports` check
//    below, applied by every implementation-provided backend factory and by
//    hosts vetting a loaded node's descriptor).

namespace xronos::abi::inline v1 {

inline constexpr std::uint32_t version_major = 1;
inline constexpr std::uint32_t version_minor = 0;

// All ABI types live in the inline namespace `v1`, which encodes the major
// version: qualified names stay `xronos::abi::X`, but mangled names and
// typeinfo carry the major, so different majors are structurally unrelated
// types (a gate bug cannot silently mix their vtable layouts), and a future
// host can implement two majors side by side. A major bump renames the
// inline namespace together with this constant.
static_assert(version_major == 1, "keep the inline namespace (v1) in sync with version_major");

// The acceptance rule, evaluated by an implementation against the ABI
// version a consumer was compiled with (this header's constants, as seen at
// the implementation's own compile time).
[[nodiscard]] constexpr auto supports(std::uint32_t consumer_major, std::uint32_t consumer_minor) noexcept -> bool {
  return consumer_major == version_major && consumer_minor <= version_minor;
}

} // namespace xronos::abi::inline v1

#endif // XRONOS_ABI_VERSION_HH
