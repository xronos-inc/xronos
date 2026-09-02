// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_BACKEND_ENGINE_HH
#define XRONOS_BACKEND_ENGINE_HH

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/abi/types.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"

// The shared implementation of the SDK/implementation ABI. The engine
// realizes `abi::Backend` and `abi::RuntimeBackend` over the reactor model,
// telemetry, source-location, and validator subsystems. It owns the model
// and the lifecycle state, and it executes the program on the runtime the
// host hands to `run`.

namespace xronos::backend {

namespace detail {
struct State;
} // namespace detail

class Engine final : private abi::Backend {
public:
  Engine();
  ~Engine();
  Engine(const Engine&) = delete;
  Engine(Engine&&) = delete;
  auto operator=(const Engine&) = delete;
  auto operator=(Engine&&) = delete;

  // The engine's frozen cross-boundary view. Valid until the engine is
  // destroyed. The base is private so that an `Engine&` never converts
  // implicitly, which keeps handing out the ABI a deliberate act.
  [[nodiscard]] auto abi() noexcept -> abi::Backend& { return static_cast<abi::Backend&>(*this); }

  void enable_telemetry(std::string_view application_name, std::string_view endpoint);

  // Adds a connection that takes part in a node boundary crossing.
  void add_cross_boundary_connection(std::uint64_t from, std::uint64_t to, core::BoundaryCrossing crossing,
                                     std::optional<abi::Duration> delay = std::nullopt);

  // Completes assembly of the program. May only be called once, after all
  // elements are registered.
  void assemble();

  // Checks the assembled program and returns error messages. An empty
  // result means the model is valid. Requires `assemble` (throws
  // `std::logic_error` otherwise). May be called again after further
  // changes. A verdict covers the model exactly as checked, so any later
  // change outdates it and `run` requires a fresh passing call.
  [[nodiscard]] auto validate() -> std::vector<std::string>;

  // Exports the diagram of the assembled model. Requires `assemble` (throws
  // `std::logic_error` otherwise). A no-op when the library was built
  // without diagram support.
  void export_diagram();

  // Runs the program to completion (blocking). Requires `assemble` and a
  // `validate` that returned empty on the model as it stands now (throws
  // `std::logic_error` otherwise) and may only be called once.
  void run(std::unique_ptr<runtime::Runtime> runtime, const runtime::ExecutionProperties& properties);

  // Requests a prompt stop of a blocking `run` from any thread: reactions
  // stop dispatching, shutdown reactions run, and `run` returns normally.
  // A request before the run is prepared is latched and honored once the
  // run starts; a call after `run` returned is a harmless no-op.
  void request_stop() noexcept;

  // Read access to the assembled program. Each accessor throws
  // `std::logic_error` before `assemble`, because the model does not
  // describe the complete program until the assemble callbacks have run.
  [[nodiscard]] auto model() const -> const core::ReactorModel&;
  [[nodiscard]] auto attribute_manager() const -> const telemetry::AttributeManager&;
  [[nodiscard]] auto source_location_registry() const -> const source_location::SourceLocationRegistry&;

private:
  // The `abi::Backend` implementation, reached through `abi()`.
  [[nodiscard]] auto runtime_backend() noexcept -> abi::RuntimeBackend& final;
  [[nodiscard]] auto register_reactor(const std::string& name, abi::ElementUid parent) -> abi::ElementUid final;
  [[nodiscard]] auto register_top_level_reactor(const std::string& name) -> abi::ElementUid final;
  [[nodiscard]] auto register_input_port(const std::string& name, abi::ElementUid parent) -> abi::ElementUid final;
  [[nodiscard]] auto register_output_port(const std::string& name, abi::ElementUid parent) -> abi::ElementUid final;
  void set_port_serializer(abi::ElementUid port, abi::PortSerializer* serializer) final;
  [[nodiscard]] auto register_periodic_timer(const std::string& name, abi::ElementUid parent, abi::Duration offset,
                                             abi::Duration period) -> abi::ElementUid final;
  void set_periodic_timer_offset(abi::ElementUid timer, abi::Duration offset) final;
  void set_periodic_timer_period(abi::ElementUid timer, abi::Duration period) final;
  [[nodiscard]] auto register_programmable_timer(const std::string& name, abi::ElementUid parent)
      -> abi::ElementUid final;
  [[nodiscard]] auto register_physical_event(const std::string& name, abi::ElementUid parent) -> abi::ElementUid final;
  [[nodiscard]] auto register_startup(const std::string& name, abi::ElementUid parent) -> abi::ElementUid final;
  [[nodiscard]] auto register_shutdown(const std::string& name, abi::ElementUid parent) -> abi::ElementUid final;
  [[nodiscard]] auto register_reaction(const std::string& name, abi::ElementUid parent, abi::ReactionHandler* handler,
                                       std::uint32_t position) -> abi::ElementUid final;
  [[nodiscard]] auto register_reaction_with_deadline(const std::string& name, abi::ElementUid parent,
                                                     abi::ReactionHandler* handler, std::uint32_t position,
                                                     abi::Duration deadline) -> abi::ElementUid final;
  void register_reaction_trigger(abi::ElementUid reaction, abi::ElementUid element) final;
  void register_reaction_effect(abi::ElementUid reaction, abi::ElementUid element) final;
  [[nodiscard]] auto register_metric(const std::string& name, abi::ElementUid parent, const std::string& description,
                                     const std::string& unit) -> abi::ElementUid final;
  void add_connection(abi::ElementUid from, abi::ElementUid to) final;
  void add_delayed_connection(abi::ElementUid from, abi::ElementUid to, abi::Duration delay) final;
  void register_assemble_callback(abi::ElementUid reactor, abi::AssembleCallback* callback) final;
  void unregister_assemble_callback(abi::ElementUid reactor) final;
  [[nodiscard]] auto add_attribute(abi::ElementUid element, const std::string& key, const std::string& value)
      -> bool final;
  [[nodiscard]] auto add_attribute(abi::ElementUid element, const std::string& key, bool value) -> bool final;
  [[nodiscard]] auto add_attribute(abi::ElementUid element, const std::string& key, std::int64_t value) -> bool final;
  [[nodiscard]] auto add_attribute(abi::ElementUid element, const std::string& key, double value) -> bool final;
  void register_source_location(abi::ElementUid element, const abi::SourceLocation& location) final;
  [[nodiscard]] auto element_fqn(abi::ElementUid element) const -> std::string final;
  void export_port(abi::ElementUid node, abi::ElementUid port, const std::string& value_type,
                   const std::string& encoding, abi::PortSerializer* serializer) final;

  [[nodiscard]] auto register_reaction_impl(const std::string& name, abi::ElementUid parent,
                                            std::unique_ptr<abi::ReactionHandler> handler, std::uint32_t position,
                                            std::optional<abi::Duration> deadline) -> abi::ElementUid;
  auto register_element(std::string_view name, core::ElementType type, std::optional<abi::ElementUid> parent)
      -> abi::ElementUid;
  void add_connection_impl(abi::ElementUid from, abi::ElementUid to, std::optional<abi::Duration> delay,
                           core::BoundaryCrossing crossing);

  // A verdict covers the model exactly as validate saw it, so every mutation
  // of the model -- element registration, property changes, reaction
  // dependencies, connections -- outdates it; run then requires a fresh
  // passing validate. Nothing ever removes elements, so mutators are the
  // only callers.
  void outdate_validation() noexcept { validated_ = false; }

  std::shared_ptr<detail::State> state_;
  bool assembled_{false};
  bool validated_{false};
  bool run_started_{false};
  bool telemetry_enabled_{false};
  std::string telemetry_application_name_{};
  std::string telemetry_endpoint_{};
};

} // namespace xronos::backend

#endif // XRONOS_BACKEND_ENGINE_HH
