// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_FWD_HH
#define XRONOS_SDK_FWD_HH

namespace xronos {

namespace telemetry {

class AttributeManager;
class MetricDataLoggerProvider;
class TelemetryBackend;

} // namespace telemetry

namespace runtime {

class Environment;
class LogicalAction;
class InputPort;
class OutputPort;
class PhysicalAction;
class Port;
class Reaction;
class ReactorElement;
class Reactor;
class Timer;

} // namespace runtime

namespace sdk {

class Element;
class Environment;
class Reactor;
class BaseReaction;
class PeriodicTimer;
template <class T> class Port;
template <class T> class InputPort;
template <class T> class OutputPort;
template <class T> class ProgrammableTimer;
template <class T> class PhysicalEvent;

} // namespace sdk

} // namespace xronos

#endif // XRONOS_SDK_FWD_HH
