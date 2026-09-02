// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/backend/engine.hh"

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/abi/exceptions.hh"
#include "xronos/abi/types.hh"
#include "xronos/abi/value.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/telemetry.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"
#include "xronos/validator/checks.hh"

namespace xronos::backend {

namespace {

auto create_telemetry_backend([[maybe_unused]] const telemetry::AttributeManager& attribute_manager,
                              [[maybe_unused]] const core::ElementRegistry& element_registry,
                              [[maybe_unused]] std::string_view application_name,
                              [[maybe_unused]] std::string_view endpoint)
    -> std::unique_ptr<telemetry::TelemetryBackend>;
void send_reactor_graph([[maybe_unused]] const core::ReactorModel& model,
                        [[maybe_unused]] const telemetry::AttributeManager& attribute_manager,
                        [[maybe_unused]] const source_location::SourceLocationRegistry& source_location_registry);

class RuntimeBackendImpl;

} // namespace

namespace detail {

// State shared between the `Engine` and the `RuntimeBackendImpl` facade.
struct State {
  core::ReactorModel model{};
  telemetry::AttributeManager attribute_manager{};
  telemetry::MetricDataLoggerProvider metric_data_logger_provider{};
  std::unique_ptr<telemetry::TelemetryBackend> telemetry_backend{std::make_unique<telemetry::NoopTelemetryBackend>()};
  source_location::SourceLocationRegistry source_location_registry{};
  // Ordered by uid: uids increase monotonically with registration, so
  // iterating the map runs the callbacks in registration order.
  std::map<abi::ElementUid, std::unique_ptr<abi::AssembleCallback>> assemble_callbacks{};
  std::unique_ptr<runtime::Runtime> runtime{};
  std::unique_ptr<runtime::ProgramHandle> program_handle{};
  std::unique_ptr<RuntimeBackendImpl> runtime_backend{};
};

} // namespace detail

namespace {

using detail::State;

class MetricRecorderImpl final : public abi::MetricRecorder {
public:
  MetricRecorderImpl(const core::Element& element, const runtime::TimeAccess& time_access,
                     telemetry::MetricDataLoggerProvider& provider)
      : element_{element}
      , time_access_{time_access}
      , provider_{provider} {}

  void record(double value) noexcept final {
    provider_.get().logger().record(element_.get(), time_access_.get(), value);
  }
  void record(std::int64_t value) noexcept final {
    provider_.get().logger().record(element_.get(), time_access_.get(), value);
  }

private:
  std::reference_wrapper<const core::Element> element_;
  std::reference_wrapper<const runtime::TimeAccess> time_access_;
  std::reference_wrapper<telemetry::MetricDataLoggerProvider> provider_;
};

class RuntimeBackendImpl final : public abi::RuntimeBackend {
public:
  explicit RuntimeBackendImpl(State& state)
      : state_{state} {}

  // Called by run() once the program is prepared, before execution starts.
  void publish(runtime::ProgramHandle* handle) noexcept { handle_.store(handle, std::memory_order_seq_cst); }

  void latch_stop_request() noexcept { stop_requested_.store(true, std::memory_order_seq_cst); }
  [[nodiscard]] auto stop_request_latched() const noexcept -> bool {
    return stop_requested_.load(std::memory_order_seq_cst);
  }

  // Permanently closes the trigger path: every later trigger_physical_event
  // call reports Stopped without touching the program. Closing has two
  // stages. Setting `closing_` rejects new calls before they touch the
  // mutex; a reader-preferring rwlock (glibc's default) admits new shared
  // lockers while a writer waits, so without this stage a sustained stream
  // of triggers could starve the drain. Taking the unique lock then waits
  // out the bounded set of deliveries already holding the shared lock, so
  // returning from retire() guarantees that no trigger call still reaches
  // into the program. noexcept: locking can in principle throw
  // std::system_error, and at teardown terminating is preferable to
  // propagating.
  void retire() noexcept {
    closing_.store(true, std::memory_order_release);
    const std::unique_lock drain{gate_mutex_};
  }

  // The prepared program, or nullptr before run() publishes it. The handle
  // stays alive in State for the engine's whole lifetime, so a caller on any
  // thread may use the returned pointer while the engine exists.
  [[nodiscard]] auto prepared_run() const noexcept -> runtime::ProgramHandle* {
    return handle_.load(std::memory_order_seq_cst);
  }
  // The ABI interface deliberately has a protected, non-virtual destructor
  // (implementation-owned); this concrete class is destroyed as itself, via
  // State's unique_ptr. Retiring here is a backstop for runs that never
  // reach run()'s own retire; State destroys this facade before the program
  // handle, so the drain still precedes the memory it protects.
  virtual ~RuntimeBackendImpl() { retire(); }
  RuntimeBackendImpl(const RuntimeBackendImpl&) = delete;
  RuntimeBackendImpl(RuntimeBackendImpl&&) = delete;
  auto operator=(const RuntimeBackendImpl&) = delete;
  auto operator=(RuntimeBackendImpl&&) = delete;

  [[nodiscard]] auto get_trigger(abi::ElementUid reaction, abi::ElementUid trigger) const noexcept
      -> const abi::GettableTrigger* final {
    auto* handle = prepared_run();
    return handle == nullptr ? nullptr : handle->get_trigger(reaction, trigger);
  }
  [[nodiscard]] auto get_settable_effect(abi::ElementUid reaction, abi::ElementUid effect) noexcept
      -> abi::SettableEffect* final {
    auto* handle = prepared_run();
    return handle == nullptr ? nullptr : handle->get_settable_effect(reaction, effect);
  }
  [[nodiscard]] auto get_schedulable_effect(abi::ElementUid reaction, abi::ElementUid effect) noexcept
      -> abi::SchedulableEffect* final {
    auto* handle = prepared_run();
    return handle == nullptr ? nullptr : handle->get_schedulable_effect(reaction, effect);
  }
  [[nodiscard]] auto get_shutdown_effect(abi::ElementUid reaction, abi::ElementUid effect) noexcept
      -> abi::ShutdownEffect* final {
    auto* handle = prepared_run();
    return handle == nullptr ? nullptr : handle->get_shutdown_effect(reaction, effect);
  }
  [[nodiscard]] auto get_time_access(abi::ElementUid reactor) const noexcept -> const abi::TimeAccess* final {
    auto* handle = prepared_run();
    return handle == nullptr ? nullptr : handle->get_time_access(reactor);
  }
  [[nodiscard]] auto get_external_trigger(abi::ElementUid physical_event) noexcept -> abi::ExternalTrigger* final {
    auto* handle = prepared_run();
    return handle == nullptr ? nullptr : handle->get_external_trigger(physical_event);
  }
  [[nodiscard]] auto trigger_physical_event(abi::ElementUid physical_event, abi::AnyValue&& value) noexcept
      -> abi::TriggerStatus final {
    // `closing_` is checked before taking the shared lock, so a closed gate
    // rejects without touching the mutex, and rechecked under it, so a call
    // that raced the flip still rejects. Rejecting paths return without
    // moving from `value`: the payload dies in the caller's frame after the
    // lock is released, so its destructor can never deadlock against the
    // drain.
    if (closing_.load(std::memory_order_acquire)) {
      return abi::TriggerStatus::Stopped;
    }
    // The shared lock spans the whole delivery, so retire() waits for it
    // (see retire above).
    const std::shared_lock lock{gate_mutex_};
    if (closing_.load(std::memory_order_acquire)) {
      return abi::TriggerStatus::Stopped;
    }
    auto* handle = prepared_run();
    if (handle == nullptr) {
      return abi::TriggerStatus::NotStarted;
    }
    auto* trigger = handle->get_external_trigger(physical_event);
    if (trigger == nullptr) {
      return abi::TriggerStatus::UnknownPhysicalEvent;
    }
    return trigger->try_trigger(std::move(value));
  }

  [[nodiscard]] auto get_metric_recorder(abi::ElementUid metric) noexcept -> abi::MetricRecorder* final {
    auto* handle = prepared_run();
    if (handle == nullptr) {
      return nullptr;
    }
    const std::lock_guard lock{metric_recorders_mutex_};
    auto it = metric_recorders_.find(metric);
    if (it != metric_recorders_.end()) {
      return it->second.get();
    }
    const auto& element = state_.model.element_registry.get(metric);
    const auto* time_access = handle->get_time_access(element.parent_uid.value());
    if (time_access == nullptr) {
      return nullptr;
    }
    auto recorder = std::make_unique<MetricRecorderImpl>(element, *time_access, state_.metric_data_logger_provider);
    return metric_recorders_.emplace(metric, std::move(recorder)).first->second.get();
  }

private:
  State& state_;
  std::atomic<runtime::ProgramHandle*> handle_{nullptr};
  std::atomic<bool> stop_requested_{false};
  // The live/dead gate of the trigger path (see retire and
  // trigger_physical_event). Dead is terminal: a backend runs at most once,
  // so the gate never re-arms.
  std::shared_mutex gate_mutex_{};
  std::atomic<bool> closing_{false};
  // The SDK looks each recorder up only once, so this lock is off the hot
  // path; the map keeps recorders alive as long as this RuntimeBackend.
  std::mutex metric_recorders_mutex_{};
  std::unordered_map<abi::ElementUid, std::unique_ptr<MetricRecorderImpl>> metric_recorders_{};
};

// Wraps the registered handler with the execution guard and the
// reaction-span telemetry scope.
class InstrumentedReactionHandler final : public abi::ReactionHandler {
public:
  InstrumentedReactionHandler(State& state, std::unique_ptr<abi::ReactionHandler> inner, abi::ElementUid uid)
      : state_{state}
      , inner_{std::move(inner)}
      , uid_{uid} {}

  void invoke() final {
    if (state_.get().program_handle == nullptr) {
      throw abi::ValidationError{"Reaction called without a valid execution context."};
    }
    auto scope =
        state_.get().telemetry_backend->reaction_span_logger().record_reaction_span(uid_, *state_.get().program_handle);
    inner_->invoke();
  }

private:
  std::reference_wrapper<State> state_;
  std::unique_ptr<abi::ReactionHandler> inner_;
  abi::ElementUid uid_;
};

} // namespace

Engine::Engine()
    : state_{std::make_shared<State>()} {
  // The lookup facade exists for the engine's whole lifetime. Its lookups
  // return nullptr until run() publishes the prepared program.
  state_->runtime_backend = std::make_unique<RuntimeBackendImpl>(*state_);
}

Engine::~Engine() = default;

auto Engine::runtime_backend() noexcept -> abi::RuntimeBackend& { return *state_->runtime_backend; }

auto Engine::register_reactor(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::ReactorTag{}, parent);
}
auto Engine::register_top_level_reactor(const std::string& name) -> abi::ElementUid {
  return register_element(name, core::ReactorTag{}, std::nullopt);
}
auto Engine::register_input_port(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::InputPortTag{std::make_unique<core::PortProperties>()}, parent);
}
auto Engine::register_output_port(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::OutputPortTag{std::make_unique<core::PortProperties>()}, parent);
}

// The setter adopts the callback BEFORE the element lookup, which can
// throw: ownership transfers immediately even then (see abi/backend.hh).
void Engine::set_port_serializer(abi::ElementUid port, abi::PortSerializer* serializer) {
  auto owned = std::unique_ptr<abi::PortSerializer>{serializer};
  core::get_port_properties(state_->model.element_registry.get(port)).serializer = std::move(owned);
  outdate_validation();
}

void Engine::export_port(abi::ElementUid node, abi::ElementUid port, const std::string& value_type,
                         const std::string& encoding, abi::PortSerializer* serializer) {
  // The callback is adopted before the checks below, like the setter above:
  // ownership transfers on the call.
  auto owned_serializer = std::unique_ptr<abi::PortSerializer>{serializer};

  // Exporting declares an identity and promises the port can be encoded
  // under it, so unlike `set_port_serializer` -- where a null simply leaves
  // the port unserialized -- a null here would declare a promise the port
  // cannot keep, and would surface only once something tried to use it.
  if (owned_serializer == nullptr) {
    throw std::invalid_argument{"export_port requires a non-null serializer."};
  }
  // An empty name identifies nothing, so two ports carrying unrelated types
  // or encodings would compare as a match. Checked before the element
  // lookups, since this is a malformed argument rather than a bad model.
  if (value_type.empty()) {
    throw std::invalid_argument{"export_port requires a non-empty value type name."};
  }
  if (encoding.empty()) {
    throw std::invalid_argument{"export_port requires a non-empty encoding name."};
  }

  const auto& registry = state_->model.element_registry;
  const auto& port_element = registry.get(port);
  if (port_element.parent_uid != node) {
    throw abi::ValidationError{"Cannot export port " + port_element.fqn + " from " + registry.get(node).fqn +
                               " because it is not one of its own ports. A node exports ports that belong to it; "
                               "to export a child reactor's port, connect it to a port of the node itself."};
  }
  auto& properties = core::get_port_properties(port_element);
  if (properties.export_info.has_value()) {
    throw abi::ValidationError{"Port " + port_element.fqn + " is already exported."};
  }
  // This serializer describes the encoding the port is exported under, so
  // it replaces any the port already carries.
  properties.serializer = std::move(owned_serializer);
  properties.export_info = core::ExportInfo{.value_type = value_type, .encoding = encoding};
  outdate_validation();
}

auto Engine::register_periodic_timer(const std::string& name, abi::ElementUid parent, abi::Duration offset,
                                     abi::Duration period) -> abi::ElementUid {
  return register_element(name,
                          core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>(
                              core::PeriodicTimerProperties{.offset = offset, .period = period})},
                          parent);
}
void Engine::set_periodic_timer_offset(abi::ElementUid timer, abi::Duration offset) {
  core::get_properties<core::PeriodicTimerTag>(state_->model.element_registry.get(timer)).offset = offset;
  outdate_validation();
}
void Engine::set_periodic_timer_period(abi::ElementUid timer, abi::Duration period) {
  core::get_properties<core::PeriodicTimerTag>(state_->model.element_registry.get(timer)).period = period;
  outdate_validation();
}
auto Engine::register_programmable_timer(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::ProgrammableTimerTag{}, parent);
}
auto Engine::register_physical_event(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::PhysicalEventTag{}, parent);
}
auto Engine::register_startup(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::StartupTag{}, parent);
}
auto Engine::register_shutdown(const std::string& name, abi::ElementUid parent) -> abi::ElementUid {
  return register_element(name, core::ShutdownTag{}, parent);
}

auto Engine::register_reaction(const std::string& name, abi::ElementUid parent, abi::ReactionHandler* handler,
                               std::uint32_t position) -> abi::ElementUid {
  return register_reaction_impl(name, parent, std::unique_ptr<abi::ReactionHandler>{handler}, position, std::nullopt);
}
auto Engine::register_reaction_with_deadline(const std::string& name, abi::ElementUid parent,
                                             abi::ReactionHandler* handler, std::uint32_t position,
                                             abi::Duration deadline) -> abi::ElementUid {
  return register_reaction_impl(name, parent, std::unique_ptr<abi::ReactionHandler>{handler}, position, deadline);
}

auto Engine::register_reaction_impl(const std::string& name, abi::ElementUid parent,
                                    std::unique_ptr<abi::ReactionHandler> handler, std::uint32_t position,
                                    std::optional<abi::Duration> deadline) -> abi::ElementUid {
  if (handler == nullptr) {
    throw std::invalid_argument{"register_reaction requires a non-null handler."};
  }
  auto uid = register_element(name,
                              core::ReactionTag{std::make_unique<core::ReactionProperties>(core::ReactionProperties{
                                  .handler = nullptr, .position = position, .deadline = deadline})},
                              parent);
  // The wrapper needs the reaction's uid for span logging, so the handler
  // is set after registration assigned it.
  core::get_properties<core::ReactionTag>(state_->model.element_registry.get(uid)).handler =
      std::make_unique<InstrumentedReactionHandler>(*state_, std::move(handler), uid);
  return uid;
}

void Engine::register_reaction_trigger(abi::ElementUid reaction, abi::ElementUid element) {
  // Triggers registered while the assemble callbacks run (inside assemble)
  // are fine; only a prepared program is sealed.
  if (state_->program_handle != nullptr) {
    throw abi::ValidationError{"Triggers may not be declared once execution has started."};
  }
  if (auto error = validator::check_trigger_hierarchy(state_->model.element_registry, reaction, element)) {
    throw abi::ValidationError{*error};
  }
  state_->model.reaction_dependency_registry.register_reaction_trigger(reaction, element);
  outdate_validation();
}
void Engine::register_reaction_effect(abi::ElementUid reaction, abi::ElementUid element) {
  // Effects registered while the assemble callbacks run (inside assemble)
  // are fine; only a prepared program is sealed.
  if (state_->program_handle != nullptr) {
    throw abi::ValidationError{"Effects may not be declared once execution has started."};
  }
  if (auto error = validator::check_effect_hierarchy(state_->model.element_registry, reaction, element)) {
    throw abi::ValidationError{*error};
  }
  state_->model.reaction_dependency_registry.register_reaction_effect(reaction, element);
  outdate_validation();
}

auto Engine::register_metric(const std::string& name, abi::ElementUid parent, const std::string& description,
                             const std::string& unit) -> abi::ElementUid {
  return register_element(name,
                          core::MetricTag{std::make_unique<core::MetricProperties>(
                              core::MetricProperties{.description = description, .unit = unit})},
                          parent);
}

void Engine::add_connection(abi::ElementUid from, abi::ElementUid to) {
  add_connection_impl(from, to, std::nullopt, core::BoundaryCrossing::None);
}
void Engine::add_delayed_connection(abi::ElementUid from, abi::ElementUid to, abi::Duration delay) {
  add_connection_impl(from, to, delay, core::BoundaryCrossing::None);
}
void Engine::add_cross_boundary_connection(abi::ElementUid from, abi::ElementUid to, core::BoundaryCrossing crossing,
                                           std::optional<abi::Duration> delay) {
  util::assert_(crossing != core::BoundaryCrossing::None);
  add_connection_impl(from, to, delay, crossing);
}

void Engine::add_connection_impl(abi::ElementUid from, abi::ElementUid to, std::optional<abi::Duration> delay,
                                 core::BoundaryCrossing crossing) {
  // Connections registered while the assemble callbacks run (inside
  // assemble) are fine; only a prepared program is sealed.
  if (state_->program_handle != nullptr) {
    throw abi::ValidationError{"Connections may not be created once execution has started."};
  }
  if (auto error = validator::check_connection_hierarchy(state_->model.element_registry, from, to)) {
    throw abi::ValidationError{*error};
  }
  auto& connection_graph = state_->model.connection_graph;
  if (connection_graph.has_incoming_connection(to)) {
    const auto& registry = state_->model.element_registry;
    auto upstream_uid = connection_graph.get_upstream_uid(to).value();
    throw abi::ValidationError{"Cannot connect port " + registry.get(from).fqn + " to port " + registry.get(to).fqn +
                               " because it already has an inbound connection from port " +
                               registry.get(upstream_uid).fqn + ". Each port may have at most one inbound " +
                               "connection."};
  }
  auto success =
      connection_graph.add_connection({.from_uid = from, .to_uid = to, .delay = delay, .crossing = crossing});
  util::assert_(success);
  outdate_validation();
}

void Engine::register_assemble_callback(abi::ElementUid reactor, abi::AssembleCallback* callback) {
  auto result = state_->assemble_callbacks.try_emplace(reactor, std::unique_ptr<abi::AssembleCallback>{callback});
  util::assert_(result.second);
}
void Engine::unregister_assemble_callback(abi::ElementUid reactor) {
  auto result = state_->assemble_callbacks.erase(reactor);
  util::assert_(result == 1);
}

auto Engine::add_attribute(abi::ElementUid element, const std::string& key, const std::string& value) -> bool {
  return state_->attribute_manager.add_attribute(element, key, telemetry::AttributeValue{value});
}
auto Engine::add_attribute(abi::ElementUid element, const std::string& key, bool value) -> bool {
  return state_->attribute_manager.add_attribute(element, key, telemetry::AttributeValue{value});
}
auto Engine::add_attribute(abi::ElementUid element, const std::string& key, std::int64_t value) -> bool {
  return state_->attribute_manager.add_attribute(element, key, telemetry::AttributeValue{value});
}
auto Engine::add_attribute(abi::ElementUid element, const std::string& key, double value) -> bool {
  return state_->attribute_manager.add_attribute(element, key, telemetry::AttributeValue{value});
}

void Engine::register_source_location(abi::ElementUid element, const abi::SourceLocation& location) {
  state_->source_location_registry.add_source_location(element, source_location::SourceLocation{
                                                                    .file = std::string{location.file},
                                                                    .function = std::string{location.function},
                                                                    .start_line = location.start_line,
                                                                    .end_line = location.end_line,
                                                                    .start_column = location.start_column,
                                                                    .end_column = location.end_column,
                                                                });
}

auto Engine::element_fqn(abi::ElementUid element) const -> std::string {
  return state_->model.element_registry.get(element).fqn;
}

void Engine::enable_telemetry(std::string_view application_name, std::string_view endpoint) {
  telemetry_enabled_ = true;
  telemetry_application_name_ = std::string{application_name};
  telemetry_endpoint_ = std::string{endpoint};
}

void Engine::export_diagram() {
  if (!assembled_) {
    throw std::logic_error{"assemble must be called before export_diagram."};
  }
  send_reactor_graph(state_->model, state_->attribute_manager, state_->source_location_registry);
}

auto Engine::model() const -> const core::ReactorModel& {
  if (!assembled_) {
    throw std::logic_error{"assemble must be called before model."};
  }
  return state_->model;
}

auto Engine::attribute_manager() const -> const telemetry::AttributeManager& {
  if (!assembled_) {
    throw std::logic_error{"assemble must be called before attribute_manager."};
  }
  return state_->attribute_manager;
}

auto Engine::source_location_registry() const -> const source_location::SourceLocationRegistry& {
  if (!assembled_) {
    throw std::logic_error{"assemble must be called before source_location_registry."};
  }
  return state_->source_location_registry;
}

void Engine::assemble() {
  if (std::exchange(assembled_, true)) {
    throw std::logic_error{"assemble may only be called once."};
  }

  // Run the assemble callbacks in registration order. Callbacks may
  // register further elements and callbacks, so advance by key instead of
  // holding an iterator across calls.
  std::optional<abi::ElementUid> last{};
  while (true) {
    auto it = last.has_value() ? state_->assemble_callbacks.upper_bound(*last) : state_->assemble_callbacks.begin();
    if (it == state_->assemble_callbacks.end()) {
      break;
    }
    last = it->first;
    it->second->invoke();
  }
}

auto Engine::validate() -> std::vector<std::string> {
  if (!assembled_) {
    throw std::logic_error{"assemble must be called before validate."};
  }
  // The checks see the model as it is now, including anything added since
  // assembly. The verdict holds only while the model stays unchanged:
  // every mutation clears `validated_` again (see outdate_validation).
  auto result = validator::run_all_checks(state_->model);
  validated_ = result.has_value();
  return validated_ ? std::vector<std::string>{} : std::move(result.error());
}

void Engine::run(std::unique_ptr<runtime::Runtime> runtime, const runtime::ExecutionProperties& properties) {
  // Checked before anything else, and in particular before the one-shot
  // run flag below: a malformed call must not consume the run.
  if (runtime == nullptr) {
    throw std::invalid_argument{"run requires a non-null runtime."};
  }
  if (!assembled_) {
    throw std::logic_error{"assemble must be called before run."};
  }
  if (!validated_) {
    throw std::logic_error{
        "run requires a passing validate on the current model: call validate after the last change to the model, "
        "and resolve any errors it reports."};
  }
  if (std::exchange(run_started_, true)) {
    throw std::logic_error{
        "run may only be called once. To correctly start a new instance of the program, create a new backend."};
  }

  if (telemetry_enabled_) {
    state_->telemetry_backend = create_telemetry_backend(state_->attribute_manager, state_->model.element_registry,
                                                         telemetry_application_name_, telemetry_endpoint_);
    state_->metric_data_logger_provider.set_logger(state_->telemetry_backend->metric_data_logger());
  }
  state_->telemetry_backend->initialize();

  state_->runtime = std::move(runtime);
  state_->program_handle = state_->runtime->initialize_reactor_program(state_->model, properties);

  // Publish the prepared run behind the facade BEFORE the blocking run
  // starts, so reactions (and external threads) resolving their
  // dependencies during the run find it.
  state_->runtime_backend->publish(state_->program_handle.get());

  // If a stop was requested before the publish above, then the stop is latched.
  // We forward the request to stop the run.
  if (state_->runtime_backend->stop_request_latched()) {
    state_->program_handle->request_stop();
  }

  // Retire the facade's trigger path as soon as execute() returns or
  // throws: from then on external threads must observe Stopped instead of
  // reaching into a program that is about to be destroyed. Retiring also
  // drains any trigger call still in flight (see RuntimeBackendImpl).
  struct RetireGuard {
    explicit RetireGuard(RuntimeBackendImpl* facade)
        : facade_{facade} {}
    ~RetireGuard() { facade_->retire(); }
    RetireGuard(const RetireGuard&) = delete;
    RetireGuard(RetireGuard&&) = delete;
    auto operator=(const RetireGuard&) = delete;
    auto operator=(RetireGuard&&) = delete;

  private:
    RuntimeBackendImpl* facade_;
  };
  const RetireGuard retire_guard{state_->runtime_backend.get()};

  state_->program_handle->execute();
}

void Engine::request_stop() noexcept {
  // Always latch the stop request. If there is no active handle yet,
  // the latched stop will be picked up later. If there is an active handle,
  // then we forward the stop request immediately.
  state_->runtime_backend->latch_stop_request();
  auto* handle = state_->runtime_backend->prepared_run();
  if (handle != nullptr) {
    handle->request_stop();
  }
}

auto Engine::register_element(std::string_view name, core::ElementType type, std::optional<abi::ElementUid> parent)
    -> abi::ElementUid {
  auto element = state_->model.element_registry.add_new_element(name, std::move(type), parent);
  if (!element.has_value()) {
    throw abi::InvalidNameError{element.error()};
  }
  outdate_validation();
  return element->get().uid;
}

} // namespace xronos::backend

#ifdef XRONOS_BACKEND_ENABLE_TELEMETRY

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <unistd.h>

#include "xronos/telemetry/otel/otel_telemetry_backend.hh"

namespace xronos::backend {
namespace {

auto create_telemetry_backend(const telemetry::AttributeManager& attribute_manager,
                              const core::ElementRegistry& element_registry, std::string_view application_name,
                              std::string_view endpoint) -> std::unique_ptr<telemetry::TelemetryBackend> {
  constexpr std::size_t hostname_buffer_size = 128;
  std::array<char, hostname_buffer_size> hostname_buffer{};
  // Pass hostname_buffer_size - 1: POSIX does not guarantee null-termination
  // on truncation, so this keeps a terminator for the std::string conversion
  // below.
  std::string hostname{"unknown"};
  if (gethostname(hostname_buffer.data(), hostname_buffer_size - 1) != 0) {
    // On failure the buffer contents are unspecified; fall back to a clear
    // sentinel rather than emitting an empty (or garbage) hostname in
    // telemetry.
    util::log::warn() << "Failed to query hostname (" << std::strerror(errno) << "). Using \"unknown\" for telemetry.";
  } else {
    hostname = hostname_buffer.data();
  }

  return std::make_unique<telemetry::otel::OtelTelemetryBackend>(attribute_manager, element_registry, application_name,
                                                                 endpoint, hostname, getpid());
}

} // namespace
} // namespace xronos::backend

#else // XRONOS_BACKEND_ENABLE_TELEMETRY

namespace xronos::backend {
namespace {

auto create_telemetry_backend(const telemetry::AttributeManager& /*attribute_manager*/,
                              const core::ElementRegistry& /*element_registry*/, std::string_view /*application_name*/,
                              std::string_view /*endpoint*/) -> std::unique_ptr<telemetry::TelemetryBackend> {
  return std::make_unique<telemetry::NoopTelemetryBackend>();
}

} // namespace
} // namespace xronos::backend

#endif // XRONOS_BACKEND_ENABLE_TELEMETRY

#ifdef XRONOS_BACKEND_ENABLE_DIAGRAMS

#include "xronos/graph_exporter/exporter.hh"

namespace xronos::backend {
namespace {

void send_reactor_graph(const core::ReactorModel& model, const telemetry::AttributeManager& attribute_manager,
                        const source_location::SourceLocationRegistry& source_location_registry) {
  graph_exporter::send_reactor_graph_to_diagram_server(model, attribute_manager, source_location_registry);
}

} // namespace
} // namespace xronos::backend

#else // XRONOS_BACKEND_ENABLE_DIAGRAMS

namespace xronos::backend {
namespace {

void send_reactor_graph(const core::ReactorModel& /*model*/, const telemetry::AttributeManager& /*attribute_manager*/,
                        const source_location::SourceLocationRegistry& /*source_location_registry*/) {}

} // namespace
} // namespace xronos::backend

#endif // XRONOS_BACKEND_ENABLE_DIAGRAMS
