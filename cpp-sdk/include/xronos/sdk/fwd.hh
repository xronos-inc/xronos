// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_FWD_HH
#define XRONOS_SDK_FWD_HH

namespace xronos {

namespace runtime {

struct Runtime;

} // namespace runtime

namespace sdk {

class Reactor;
class BaseReaction;
class PeriodicTimer;

namespace detail {

struct ContextAccess;
class ProgramContext;

} // namespace detail

} // namespace sdk

} // namespace xronos

#endif // XRONOS_SDK_FWD_HH
