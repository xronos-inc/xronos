// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/default_runtime.hh"

#include <algorithm>
#include <bits/ranges_algo.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/connection_properties.hh"
#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/port.hh"
#include "xronos/runtime/default/impl/reaction.hh"
#include "xronos/runtime/default/impl/reactor.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/util/assert.hh"

namespace xronos::runtime::default_ {

class DefaultProgramHandle final : public ProgramHandle {
public:
  DefaultProgramHandle(const ExecutionProperties& properties)
      : environment_{properties.num_workers, properties.fast_mode, properties.timeout} {}
  DefaultProgramHandle(const DefaultProgramHandle&) = delete;
  DefaultProgramHandle(DefaultProgramHandle&&) = delete;
  auto operator=(const DefaultProgramHandle&) = delete;
  auto operator=(DefaultProgramHandle&&) -> DefaultProgramHandle& = delete;
  ~DefaultProgramHandle() final;

  void initialize(const core::ReactorModel& model);
  void execute() override;

  void wait_until_program_terminates() final;

  [[nodiscard]] auto get_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept
      -> const GettableTrigger* final;
  [[nodiscard]] auto get_settable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> SettableEffect* final;
  [[nodiscard]] auto get_schedulable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> SchedulableEffect* final;
  [[nodiscard]] auto get_shutdown_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> ShutdownEffect* final;
  [[nodiscard]] auto get_external_trigger(std::uint64_t external_trigger_uid) noexcept -> ExternalTrigger* final {
    return &get_element<impl::PhysicalAction>(external_trigger_uid);
  }
  [[nodiscard]] auto get_time_access([[maybe_unused]] std::uint64_t reactor_uid) const noexcept
      -> const TimeAccess* final {
    return &get_element<impl::Reactor>(reactor_uid);
  }

  void rethrow_exception_if_any() const final { environment_.rethrow_exception_if_any(); }

private:
  impl::Environment environment_;
  std::unordered_map<std::uint64_t, std::unique_ptr<impl::ReactorElement>> elements_{};
  std::thread thread_{};

  class DefaultShutdownEffect final : public ShutdownEffect {
  public:
    DefaultShutdownEffect(impl::Environment& environment)
        : environment_{environment} {}

    void trigger_shutdown() noexcept final { environment_.get().sync_shutdown(); }

  private:
    std::reference_wrapper<impl::Environment> environment_;
  };

  DefaultShutdownEffect shutdown_effect_{environment_};
  std::set<std::pair<std::uint64_t, std::uint64_t>> declared_shutdown_effects_;

  class ElementInserter {
  public:
    ElementInserter(const core::Element& element, DefaultProgramHandle& handle)
        : element_{element}
        , handle_{handle} {}

    void operator()([[maybe_unused]] const core::InputPortTag& type) { insert_element<impl::Input>(); }
    void operator()([[maybe_unused]] const core::MetricTag& type) {}
    void operator()([[maybe_unused]] const core::OutputPortTag& type) { insert_element<impl::Output>(); }
    void operator()([[maybe_unused]] const core::PeriodicTimerTag& type) { insert_element<impl::Timer>(); }
    void operator()([[maybe_unused]] const core::PhysicalEventTag& type) { insert_element<impl::PhysicalAction>(); }
    void operator()([[maybe_unused]] const core::ProgrammableTimerTag& type) { insert_element<impl::LogicalAction>(); }
    void operator()([[maybe_unused]] const core::ReactionTag& type) { insert_element<impl::Reaction>(); }
    void operator()([[maybe_unused]] const core::ReactorTag& type) {
      if (element_.get().parent_uid.has_value()) {
        insert_element<impl::Reactor>();
      } else {
        handle_.get().elements_.try_emplace(element_.get().uid,
                                            std::make_unique<impl::Reactor>(element_, handle_.get().environment_));
      }
    }
    void operator()([[maybe_unused]] const core::ShutdownTag& type) { insert_element<impl::ShutdownTrigger>(); }
    void operator()([[maybe_unused]] const core::StartupTag& type) { insert_element<impl::StartupTrigger>(); }

  private:
    std::reference_wrapper<const core::Element> element_;
    std::reference_wrapper<DefaultProgramHandle> handle_;

    template <class T> void insert_element() {
      auto parent_uid = element_.get().parent_uid;
      util::assert_(parent_uid.has_value());
      handle_.get().elements_.try_emplace(
          element_.get().uid,
          std::make_unique<T>(element_, handle_.get().get_element<impl::Reactor>(parent_uid.value())));
    }
  };

  template <class T> auto get_element(std::uint64_t uid) -> T& { return dynamic_cast<T&>(*elements_.at(uid)); }
  template <class T> auto get_element(std::uint64_t uid) const -> const T& {
    return dynamic_cast<const T&>(*elements_.at(uid));
  }

  template <class T> auto try_get_element(std::uint64_t uid) -> T* { return dynamic_cast<T*>(elements_.at(uid).get()); }
  template <class T> auto try_get_element(std::uint64_t uid) const -> const T* {
    return dynamic_cast<const T*>(elements_.at(uid).get());
  }

  template <class ReturnType, class T, class... Ts> auto get_element_variant_impl(std::uint64_t uid) -> ReturnType {
    if constexpr (!sizeof...(Ts)) {
      return get_element<T>(uid);
    } else {
      if (auto* elem = try_get_element<T>(uid)) {
        return *elem;
      }
      return get_element_variant_impl<ReturnType, Ts...>(uid);
    }
  }

  template <class ReturnType, class T, class... Ts>
  auto get_element_variant_const_impl(std::uint64_t uid) const -> ReturnType {
    if constexpr (!sizeof...(Ts)) {
      return get_element<T>(uid);
    } else {
      if (auto* elem = try_get_element<T>(uid)) {
        return *elem;
      }
      return get_element_variant_const_impl<ReturnType, Ts...>(uid);
    }
  }

  template <class... T> auto get_element_variant(std::uint64_t uid) -> auto {
    return get_element_variant_impl<std::variant<std::reference_wrapper<T>...>, T...>(uid);
  }

  template <class... T> auto get_element_variant(std::uint64_t uid) const -> auto {
    return get_element_variant_const_impl<std::variant<std::reference_wrapper<const T>...>, T...>(uid);
  }
};

DefaultProgramHandle::~DefaultProgramHandle() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void DefaultProgramHandle::execute() {
  environment_.assemble();
  thread_ = environment_.startup();
}

void DefaultProgramHandle::wait_until_program_terminates() {
  util::assert_(thread_.joinable());
  thread_.join();
}

auto DefaultRuntime::initialize_reactor_program(const core::ReactorModel& model, const ExecutionProperties& properties)
    -> std::unique_ptr<ProgramHandle> {

  auto handle = std::make_unique<DefaultProgramHandle>(properties);
  handle->initialize(model);
  return handle;
}

template <class... Ts> struct Visitor : Ts... {
  using Ts::operator()...;
};
template <class... Ts> Visitor(Ts...) -> Visitor<Ts...>;

void DefaultProgramHandle::initialize(const core::ReactorModel& model) {
  auto core_elements_view = model.element_registry.elements();
  std::vector<std::reference_wrapper<const core::Element>> core_elements{core_elements_view.begin(),
                                                                         core_elements_view.end()};
  // Sort the vector of element references by fqn. This ensures that we will
  // initialize reactors before their contained elements.
  std::ranges::sort(core_elements,
                    [](auto const& elem1, auto const& elem2) { return elem1.get().fqn < elem2.get().fqn; });

  for (const core::Element& element : core_elements) {
    std::visit(ElementInserter(element, *this), element.type);
  }

  for (const auto& connection : model.connection_graph.connections()) {
    if (connection.delay.has_value()) {
      environment_.draw_connection(
          get_element<impl::Port>(connection.from_uid), get_element<impl::Port>(connection.to_uid),
          impl::ConnectionProperties{.type_ = impl::ConnectionType::Delayed, .delay_ = connection.delay.value()});
    } else {
      environment_.draw_connection(get_element<impl::Port>(connection.from_uid),
                                   get_element<impl::Port>(connection.to_uid),
                                   impl::ConnectionProperties{.type_ = impl::ConnectionType::Normal});
    }
  }

  for (const core::Element& elem : model.element_registry.elements_of_type<core::ReactionTag>()) {
    auto& reaction = get_element<impl::Reaction>(elem.uid);
    for (std::uint64_t trigger_uid : model.reaction_dependency_registry.get_triggers(elem.uid)) {
      auto trigger_variant = get_element_variant<impl::Port, impl::BaseAction>(trigger_uid);
      std::visit([&reaction](auto& trigger) { reaction.declare_trigger(&trigger.get()); }, trigger_variant);
    }
    for (std::uint64_t effect_uid : model.reaction_dependency_registry.get_effects(elem.uid)) {
      auto effect_variant = get_element_variant<impl::Port, impl::LogicalAction, impl::ShutdownTrigger>(effect_uid);
      std::visit(Visitor{[&](std::reference_wrapper<impl::Port> effect_ref) {
                           reaction.declare_antidependency(&effect_ref.get());
                         },
                         [&](std::reference_wrapper<impl::LogicalAction> effect_ref) {
                           reaction.declare_schedulable_action(&effect_ref.get());
                         },
                         [&]([[maybe_unused]] std::reference_wrapper<impl::ShutdownTrigger> effect_ref) {
                           declared_shutdown_effects_.insert(std::make_pair(elem.uid, effect_uid));
                         }},
                 effect_variant);
    }
  }
}

auto DefaultProgramHandle::get_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept
    -> const GettableTrigger* {
  auto trigger_variant = get_element_variant<impl::Port, impl::BaseAction>(trigger_uid);
  const auto& reaction = get_element<impl::Reaction>(reaction_uid);

  bool is_valid_trigger = std::visit(
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
      Visitor{[&](const impl::Port& port) { return reaction.port_triggers().contains(const_cast<impl::Port*>(&port)); },
              [&](const impl::BaseAction& action) {
                return reaction.action_triggers().contains(
                    const_cast<impl::BaseAction*>(&action)); // NOLINT(cppcoreguidelines-pro-type-const-cast)
              }},
      trigger_variant);

  if (!is_valid_trigger) {
    return nullptr;
  }

  return std::visit([](auto elem_ref) { return static_cast<const GettableTrigger*>(&elem_ref.get()); },
                    trigger_variant);
}

auto DefaultProgramHandle::get_settable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
    -> SettableEffect* {
  auto& effect = get_element<impl::Port>(effect_uid);
  const auto& reaction = get_element<impl::Reaction>(reaction_uid);

  if (!reaction.antidependencies().contains(&effect)) {
    return nullptr;
  }

  return &effect;
}

auto DefaultProgramHandle::get_schedulable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
    -> SchedulableEffect* {
  auto& effect = get_element<impl::Action>(effect_uid);
  const auto& reaction = get_element<impl::Reaction>(reaction_uid);

  if (!reaction.scheduable_actions().contains(&effect)) {
    return nullptr;
  }

  return &effect;
}

auto DefaultProgramHandle::get_shutdown_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
    -> ShutdownEffect* {
  if (!declared_shutdown_effects_.contains(std::make_pair(reaction_uid, effect_uid))) {
    return nullptr;
  }

  return &shutdown_effect_;
}

} // namespace xronos::runtime::default_
