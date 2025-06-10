// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Forward-declarations for the Xronos SDK.
 */

#ifndef XRONOS_SDK_FWD_HH
#define XRONOS_SDK_FWD_HH

#include "xronos/runtime/fwd.hh"

namespace xronos::telemetry {

class AttributeManager;
class MetricDataLoggerProvider;
class TelemetryBackend;

} // namespace xronos::telemetry

namespace xronos::sdk {

class Environment;
class Reactor;
class BaseReaction;
class PeriodicTimer;
template <class T> class Port;
template <class T> class InputPort;
template <class T> class OutputPort;
template <class T> class ProgrammableTimer;
template <class T> class PhysicalEvent;

} // namespace xronos::sdk

#endif // XRONOS_SDK_FWD_HH
