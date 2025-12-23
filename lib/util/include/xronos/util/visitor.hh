// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_UTIL_VISITOR_HH
#define XRONOS_UTIL_VISITOR_HH

namespace xronos::util {

template <class... Ts> struct Visitor : Ts... {
  using Ts::operator()...;
};
template <class... Ts> Visitor(Ts...) -> Visitor<Ts...>;

} // namespace xronos::util

#endif // XRONOS_UTIL_VISITOR_HH
