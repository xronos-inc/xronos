// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_FWD_HH
#define XRONOS_SDK_FWD_HH

namespace xronos {

namespace core {

struct Element;

} // namespace core

namespace telemetry {

class AttributeManager;
class MetricDataLoggerProvider;
class TelemetryBackend;
class Metric;

} // namespace telemetry

namespace runtime {

struct GettableTrigger;
struct SettableEffect;
struct SchedulableEffect;
struct ShutdownEffect;
struct ExternalTrigger;
struct TimeAccess;
struct Runtime;

} // namespace runtime

namespace sdk {

class Element;
class Environment;
class Reactor;
class BaseReaction;
class PeriodicTimer;
template <class T> class ProgrammableTimer;
template <class T> class PhysicalEvent;

namespace detail {

struct ContextAccess;
struct ProgramContext;

} // namespace detail

} // namespace sdk

} // namespace xronos

#endif // XRONOS_SDK_FWD_HH
