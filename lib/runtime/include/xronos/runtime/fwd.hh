// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_FWD_HH
#define XRONOS_RUNTIME_FWD_HH

#include <cstdint>
#include <functional>

namespace xronos::runtime {

class Action;
class BaseAction;
class Environment;
enum class Phase : std::uint8_t;
class Port;
class Reaction;
class Reactor;
class ReactorElement;
class ReactorElementVisitor;
class Scheduler;
class Tag;
class Timer;
class StartupTrigger;
class ShutdownTrigger;
class MiscElement;

using PortCallback = std::function<void(const Port&)>;

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_FWD_HH
