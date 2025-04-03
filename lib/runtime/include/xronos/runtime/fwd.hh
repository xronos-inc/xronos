// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_FWD_HH
#define XRONOS_RUNTIME_FWD_HH

#include <cstdint>
#include <functional>

namespace xronos::runtime {

class BaseAction;
class BasePort;
class Environment;
enum class Phase : std::uint8_t;
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

template <class T> class Action;
template <class T> class Port;

using PortCallback = std::function<void(const BasePort&)>;

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_FWD_HH
