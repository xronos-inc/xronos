// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_CONTRACT_HH
#define XRONOS_ABI_CONTRACT_HH

// Compile-time enforcement of the toolchain contract this ABI rests on, in two
// tiers. The PORTABLE checks (pointer width, nanoseconds size) hold on any LP64
// target and are always compiled. The DOMAIN checks pin the platform and
// toolchain of the current sole compatibility domain -- Linux, GCC or Clang,
// libstdc++ 11+ -- with layout probes for the libstdc++ types that cross the
// boundary directly (std::string, the std::runtime_error base). Guarantees hold
// within one domain, never across; another platform would add its own domain
// checks. Included from every ABI header, so every translation unit at the
// boundary is verified.
//
// The domain checks guard a layout disagreement between two INDEPENDENTLY built
// artifacts across a DSO boundary. A single-toolchain, from-source build has no
// such boundary (macOS), so the probes are vacuous; it may define
// XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION to skip the domain tier. The backstop
// below refuses that macro on Linux/libstdc++, where the guarantee is real.
//
// The checks catch size/alignment mismatches only; member reordering needs
// ABI-diff tooling (e.g. libabigail).

#include <chrono>
#include <stdexcept>
#include <string>

namespace xronos::abi::inline v1::detail {

// Portable checks: independent of the standard library, sound on any LP64
// target, and so compiled in every build -- including single-compile ones.
// NOLINTBEGIN(*-magic-numbers)
static_assert(sizeof(void*) == 8, "The xronos ABI is only supported on 64-bit targets.");
// nanoseconds holds a signed integer rep of at least 64 bits (int64_t on LP64),
// so its size and alignment are 8 for any stdlib on LP64; the assert pins both.
static_assert(sizeof(std::chrono::nanoseconds) == 8 && alignof(std::chrono::nanoseconds) == 8);
// NOLINTEND(*-magic-numbers)

} // namespace xronos::abi::inline v1::detail

#ifndef XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION

// Domain checks: the compile-time definition of the Linux/libstdc++ domain.

#if !defined(__GLIBCXX__)
#error "The xronos ABI requires libstdc++ on both sides of the SDK/implementation boundary."
#endif

#if defined(_GLIBCXX_RELEASE) && (_GLIBCXX_RELEASE < 11)
#error "The xronos ABI requires GCC/libstdc++ 11 or later."
#endif

// std::string's layout is enforced by libstdc++'s dual-ABI machinery (the
// __cxx11 abi_tag plus versioned out-of-line symbols in libstdc++.so), but
// only under the cxx11 ABI. Both sides of the boundary must use it; the size
// probe below would catch the old COW layout, but reject it explicitly with
// a clear message.
#if defined(_GLIBCXX_USE_CXX11_ABI) && !_GLIBCXX_USE_CXX11_ABI
#error "The xronos ABI requires the cxx11 library ABI (_GLIBCXX_USE_CXX11_ABI=1)."
#endif

#if !defined(__linux__)
#error "The xronos ABI is only supported on Linux."
#endif

namespace xronos::abi::inline v1::detail {

// Layout probes for the libstdc++ standard-library types that cross the ABI
// boundary directly. The expected values are the (stable, but not
// soname-enforced) libstdc++ layouts on LP64 targets. The literals are the
// contract, deliberately not named constants.
// NOLINTBEGIN(*-magic-numbers)
static_assert(sizeof(std::string) == 32 && alignof(std::string) == 8);
// The base of the exception types in exceptions.hh, thrown by the
// implementation and caught by name on the SDK side.
static_assert(sizeof(std::runtime_error) == 16 && alignof(std::runtime_error) == 8);
// NOLINTEND(*-magic-numbers)

} // namespace xronos::abi::inline v1::detail

#else // XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION

// Refuse the opt-out on the real domain -- Linux AND libstdc++, where
// independently built artifacts actually meet (a node `.so` and a
// differently-versioned host). A single-compile build on any other
// platform/stdlib (macOS with libc++ or libstdc++) has no such boundary and is
// honored.
#if defined(__GLIBCXX__) && defined(__linux__)
#error "XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION must not be defined on the Linux/libstdc++ domain."
#endif

#endif // XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION

#endif // XRONOS_ABI_CONTRACT_HH
