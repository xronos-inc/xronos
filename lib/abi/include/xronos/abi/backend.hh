// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_BACKEND_HH
#define XRONOS_ABI_BACKEND_HH

#include <cstddef>
#include <cstdint>
#include <string>

#include "xronos/abi/types.hh"
#include "xronos/abi/value.hh"

// The virtual-interface ABI between the header-only SDK and its
// implementation.
//
// Growth discipline (all classes in this header): APPEND-ONLY. New virtual
// methods may only ever be added after the last existing one; methods must
// never be inserted, reordered, removed, or have their signature changed
// within a major ABI version. Appending bumps `abi::version_minor`;
// everything else bumps `abi::version_major` (see version.hh).
//
// Ownership: every interface here is owned by the implementation.
// `Backend` and `RuntimeBackend` live inside the implementation's engine
// (`backend::Engine`, which every host wraps in a backend class of its
// own); consumers only ever borrow references, and the small hot-path
// interfaces remain valid until the engine is destroyed. Callback objects are the one ownership transfer ACROSS the
// boundary in this header (the opaque node object in node.hh is the
// other): both are destroyed with plain `delete` — safe across `.so`
// boundaries on Linux because glibc's allocator is process-wide.
//
// Exceptions propagate natively across all methods that are not `noexcept`;
// only the types in exceptions.hh may be caught by name on the other side.

namespace xronos::abi::inline v1 {

// -- Hot-path interfaces ----------------------------------------------------
//
// Each is obtained from `RuntimeBackend` (see the `get_*` lookups below),
// typically cached by the consumer, and then called directly on the hot
// path. They are not destructible through the interface; the implementation
// owns them.

// `get` returns the value of the event at the current tag; the empty value
// means that no event is present. Valueless events (`void` event sources)
// carry a boxed `abi::Void`, never the empty value. The reference remains
// valid until the runtime advances past the current tag.
class GettableTrigger {
public:
  [[nodiscard]] virtual auto get() const noexcept -> const AnyValue& = 0;

protected:
  ~GettableTrigger() = default;
};

// Write methods take the value by rvalue reference: a send always hands
// over a fresh box (or a copy the caller made deliberately), so the
// implementation moves it into storage -- for shared payloads that is a
// pointer steal with no reference-count traffic.
class SettableEffect {
public:
  virtual void set(AnyValue&& value) noexcept = 0;
  [[nodiscard]] virtual auto get() const noexcept -> const AnyValue& = 0;

protected:
  ~SettableEffect() = default;
};

class SchedulableEffect {
public:
  virtual void schedule(AnyValue&& value, Duration delay) noexcept = 0;

protected:
  ~SchedulableEffect() = default;
};

class ShutdownEffect {
public:
  virtual void trigger_shutdown() noexcept = 0;

protected:
  ~ShutdownEffect() = default;
};

// May be called from arbitrary threads, including threads the implementation
// knows nothing about.
//
// Retained for ABI 1.0 compatibility; newer consumers call
// `RuntimeBackend::trigger_physical_event` instead, which stays safe through
// the end of a run.
class ExternalTrigger {
public:
  // Delivers the event if the program is live; otherwise the event is
  // dropped silently.
  [[deprecated("Retained for ABI 1.0 compatibility; use RuntimeBackend::trigger_physical_event")]]
  virtual void trigger(AnyValue&& value) noexcept = 0;

protected:
  ~ExternalTrigger() = default;
};

class TimeAccess {
public:
  [[nodiscard]] virtual auto get_timestamp() const noexcept -> TimePoint = 0;
  [[nodiscard]] virtual auto get_start_timestamp() const noexcept -> TimePoint = 0;

protected:
  ~TimeAccess() = default;
};

class MetricRecorder {
public:
  virtual void record(double value) noexcept = 0;
  virtual void record(std::int64_t value) noexcept = 0;

protected:
  ~MetricRecorder() = default;
};

// Receives serialized bytes from a port serializer. Implemented by the
// implementation (which decides where the bytes go, e.g. straight into a
// transport buffer) and handed to the serializer callback for the duration
// of one serialization. Passing bytes as pointer + length keeps owned
// container types off the boundary entirely.
class ByteSink {
public:
  virtual void write(const std::byte* data, std::size_t size) = 0;

protected:
  ~ByteSink() = default;
};

// -- Callback interfaces -----------------------------------------------------
//
// Callbacks cross the boundary as virtual interfaces implemented on the SDK
// side, not as std::function objects: virtual dispatch rests on the Itanium
// vtable layout the whole ABI is built on (and that the ABI-diff tooling can
// see), rather than on libstdc++'s unexported function-manager protocol.
//
// Ownership: unlike the hot-path interfaces above, callback objects are
// OWNED by the implementation. Every register_*/set_* method taking a
// callback pointer assumes ownership immediately - including when the call
// throws - and destroys the object through its public virtual destructor
// when the backend is torn down.
//
// Exceptions thrown by callback implementations propagate natively through
// the invoking call, like any other cross-boundary exception.

class AssembleCallback {
public:
  virtual void invoke() = 0;
  virtual ~AssembleCallback() = default;
};

class ReactionHandler {
public:
  virtual void invoke() = 0;
  virtual ~ReactionHandler() = default;
};

class PortSerializer {
public:
  virtual void serialize(const AnyValue& value, ByteSink& sink) = 0;
  [[nodiscard]] virtual auto deserialize(const std::byte* data, std::size_t size) -> AnyValue = 0;
  virtual ~PortSerializer() = default;
};

// -- RuntimeBackend ----------------------------------------------------------

// The hot-path lookup surface of a program run, owned by the
// implementation and obtained from `Backend::runtime_backend()`. How and
// when a run is prepared and started is not part of the ABI: the lifecycle
// is driven by the host, which is always version-locked to its
// implementation and uses implementation-side interfaces.
class RuntimeBackend {
public:
  // Hot-path lookups. Each returns nullptr while no run is prepared, and
  // during a run if the queried dependency was not registered during
  // assembly. A non-null result synchronizes with the run's preparation:
  // everything the implementation set up for the run is visible to the
  // calling thread, on any thread. The returned pointers remain valid until
  // the `Backend` is destroyed.
  [[nodiscard]] virtual auto get_trigger(ElementUid reaction, ElementUid trigger) const noexcept
      -> const GettableTrigger* = 0;
  [[nodiscard]] virtual auto get_settable_effect(ElementUid reaction, ElementUid effect) noexcept
      -> SettableEffect* = 0;
  [[nodiscard]] virtual auto get_schedulable_effect(ElementUid reaction, ElementUid effect) noexcept
      -> SchedulableEffect* = 0;
  [[nodiscard]] virtual auto get_shutdown_effect(ElementUid reaction, ElementUid effect) noexcept
      -> ShutdownEffect* = 0;
  [[nodiscard]] virtual auto get_time_access(ElementUid reactor) const noexcept -> const TimeAccess* = 0;
  [[deprecated("Retained for ABI 1.0 compatibility; use trigger_physical_event")]] [[nodiscard]] virtual auto
  get_external_trigger(ElementUid physical_event) noexcept -> ExternalTrigger* = 0;
  [[nodiscard]] virtual auto get_metric_recorder(ElementUid metric) noexcept -> MetricRecorder* = 0;

  // Delivers an event on the physical event `physical_event` if the program
  // is live; the status reports the outcome. `Accepted` means the event is
  // queued for processing. Otherwise the event is dropped: `NotStarted`
  // before the program starts, `Stopped` once it has stopped, and
  // `UnknownPhysicalEvent` when the running program resolves the uid to no
  // physical event. Before the program starts there is nothing to resolve
  // against, so a bad uid also reports `NotStarted`. The payload is
  // consumed only on `Accepted`; a rejected payload stays in the argument
  // and is destroyed by the caller.
  //
  // Safe to call from arbitrary threads, including threads the
  // implementation knows nothing about, until the `Backend` is destroyed --
  // concurrent with startup, with the run, and with the run ending.
  // Added in ABI 1.1.
  [[nodiscard]] virtual auto trigger_physical_event(ElementUid physical_event, AnyValue&& value) noexcept
      -> TriggerStatus = 0;

protected:
  // Owned by the implementation; nothing destroys it through the interface.
  ~RuntimeBackend() = default;
};

// -- Backend -----------------------------------------------------------------

// The entry interface, obtained from an implementation-provided,
// version-gated factory. Covers assembly of the reactor program (cold path,
// one virtual method per element kind) — the operations issued by inline
// SDK glue compiled into applications and node modules. Driving the
// lifecycle (completing the model, preparing and starting a run) is not
// part of the ABI: only hosts do that, and a host is always version-locked
// to its implementation. All `register_*` methods return the registered
// element's uid and throw `InvalidNameError` if `name` already exists
// within `parent` or is invalid: element names must be non-empty and must
// not contain (ASCII) whitespace or any of `.,/*$?#@`, so that fully
// qualified names remain unambiguous dot-joined paths and stay usable as
// addresses in network key expressions and selection lists. Callback
// objects (reaction handlers, serializers, assemble callbacks) transfer
// ownership to the implementation; see the callback interfaces above.
class Backend {
public:
  // The backend's hot-path lookup facade; may be called at any time, from
  // any thread, and the result stays valid until the backend is destroyed.
  // Lookups on it return nullptr until the host has prepared a run through
  // the implementation-side lifecycle -- when (and whether) that happens is
  // deliberately not part of this contract.
  [[nodiscard]] virtual auto runtime_backend() noexcept -> RuntimeBackend& = 0;

  // Top-level reactors (those owned by the environment) have no parent and
  // use the dedicated method rather than a sentinel or optional.
  [[nodiscard]] virtual auto register_reactor(const std::string& name, ElementUid parent) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_top_level_reactor(const std::string& name) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_input_port(const std::string& name, ElementUid parent) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_output_port(const std::string& name, ElementUid parent) -> ElementUid = 0;
  // Serialized bytes cross as pointer + length: serializing writes into the
  // implementation-provided ByteSink, deserializing reads from a buffer that
  // only needs to stay valid for the duration of the call. Replaces any
  // serializer the port already carries.
  virtual void set_port_serializer(ElementUid port, PortSerializer* serializer) = 0;
  [[nodiscard]] virtual auto register_periodic_timer(const std::string& name, ElementUid parent, Duration offset,
                                                     Duration period) -> ElementUid = 0;
  virtual void set_periodic_timer_offset(ElementUid timer, Duration offset) = 0;
  virtual void set_periodic_timer_period(ElementUid timer, Duration period) = 0;
  [[nodiscard]] virtual auto register_programmable_timer(const std::string& name, ElementUid parent) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_physical_event(const std::string& name, ElementUid parent) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_startup(const std::string& name, ElementUid parent) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_shutdown(const std::string& name, ElementUid parent) -> ElementUid = 0;
  // `handler` must be non-null (ownership is assumed first; a null handler
  // is rejected with `std::invalid_argument`).
  [[nodiscard]] virtual auto register_reaction(const std::string& name, ElementUid parent, ReactionHandler* handler,
                                               std::uint32_t position) -> ElementUid = 0;
  [[nodiscard]] virtual auto register_reaction_with_deadline(const std::string& name, ElementUid parent,
                                                             ReactionHandler* handler, std::uint32_t position,
                                                             Duration deadline) -> ElementUid = 0;
  // Both throw `ValidationError` once a run has been prepared (the program
  // is sealed).
  virtual void register_reaction_trigger(ElementUid reaction, ElementUid element) = 0;
  virtual void register_reaction_effect(ElementUid reaction, ElementUid element) = 0;
  [[nodiscard]] virtual auto register_metric(const std::string& name, ElementUid parent, const std::string& description,
                                             const std::string& unit) -> ElementUid = 0;

  // Both throw `ValidationError` if `to` already has an incoming
  // connection, or once a run has been prepared (the program is sealed).
  // A delayed connection with a zero delay is not the same as an undelayed
  // connection (the message still hops a microstep), hence two methods
  // rather than a sentinel.
  virtual void add_connection(ElementUid from, ElementUid to) = 0;
  virtual void add_delayed_connection(ElementUid from, ElementUid to, Duration delay) = 0;

  // Assemble callbacks run inside `assemble`, in registration order.
  // Unregistration supports destroying a reactor before the program runs.
  virtual void register_assemble_callback(ElementUid reactor, AssembleCallback* callback) = 0;
  virtual void unregister_assemble_callback(ElementUid reactor) = 0;

  // One overload per supported attribute value type; supporting a new type
  // later means appending another overload (append-only friendly). Returns
  // false if the attribute could not be set (e.g. the key already exists).
  [[nodiscard]] virtual auto add_attribute(ElementUid element, const std::string& key, const std::string& value)
      -> bool = 0;
  [[nodiscard]] virtual auto add_attribute(ElementUid element, const std::string& key, bool value) -> bool = 0;
  [[nodiscard]] virtual auto add_attribute(ElementUid element, const std::string& key, std::int64_t value) -> bool = 0;
  [[nodiscard]] virtual auto add_attribute(ElementUid element, const std::string& key, double value) -> bool = 0;

  virtual void register_source_location(ElementUid element, const SourceLocation& location) = 0;

  [[nodiscard]] virtual auto element_fqn(ElementUid element) const -> std::string = 0;

  // Declares `port` as part of `node`'s interface so something outside the
  // node can connect to it.
  //
  // `value_type` and `encoding` are the port's identity — what it carries and
  // how those values are laid out in bytes. Both are non-empty and opaque to
  // the implementation, which only compares them for equality.
  //
  // `serializer` follows the same contract and ownership transfer as
  // `set_port_serializer`; it is non-null and replaces any serializer already
  // set on the port.
  //
  // Throws `std::invalid_argument` if `serializer` is null or either name is
  // empty (ownership is assumed first), and `ValidationError` if `port` is not
  // a child of `node`, or is already exported.
  virtual void export_port(ElementUid node, ElementUid port, const std::string& value_type, const std::string& encoding,
                           PortSerializer* serializer) = 0;

protected:
  // Owned by the implementation; nothing destroys it through the interface.
  ~Backend() = default;
};

} // namespace xronos::abi::inline v1

#endif // XRONOS_ABI_BACKEND_HH
