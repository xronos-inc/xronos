// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/detail/default_runtime_impl.hh"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <ranges>
#include <variant>

#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::runtime::default_::detail {

void DefaultRuntimeImpl::initialize(const core::ReactorModel& model) {
  util::assert_(reactor_model_ == nullptr);
  util::assert_(runtime_model_ == nullptr);
  reactor_model_ = &model;
  runtime_model_ = std::make_unique<RuntimeModel>();
  runtime_model_->init(model);

  scheduler_.init(model, *runtime_model_);

  // Populate the physical event map up front. External threads may call
  // get_physical_event concurrently, so the map must be complete and
  // immutable before the program handle is published.
  for (const auto& elem : model.element_registry.elements_of_type<core::PhysicalEventTag>()) {
    auto res = physical_events_.try_emplace(elem.uid, elem.uid, scheduler_);
    util::assert_(res.second);
  }
}

auto DefaultRuntimeImpl::get_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept
    -> const GettableTrigger* {
  if (auto it = runtime_model_->triggers.find(trigger_uid);
      it == runtime_model_->triggers.end() ||
      std::ranges::find(it->second.triggered_reaction_uids, reaction_uid) == it->second.triggered_reaction_uids.end()) {
    util::log::error() << "Requested read access to element " << reactor_model_->element_registry.get(trigger_uid).fqn
                       << " which is not a registered trigger of reaction "
                       << reactor_model_->element_registry.get(reaction_uid).fqn;
    return nullptr;
  }

  auto it = triggers_.find(trigger_uid);
  if (it == triggers_.end()) {
    auto res = triggers_.try_emplace(trigger_uid, trigger_uid, scheduler_);
    util::assert_(res.second);
    it = res.first;
  }

  return &it->second;
}

auto DefaultRuntimeImpl::get_settable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
    -> SettableEffect* {
  if (!is_valid_effect(reaction_uid, effect_uid)) {
    util::log::error() << "Requested write access to element " << reactor_model_->element_registry.get(effect_uid).fqn
                       << " which is not a registered effect of reaction "
                       << reactor_model_->element_registry.get(reaction_uid).fqn;
    return nullptr;
  }

  if (const auto& elem = reactor_model_->element_registry.get(effect_uid);
      !(std::holds_alternative<core::InputPortTag>(elem.type) ||
        std::holds_alternative<core::OutputPortTag>(elem.type))) {
    util::log::error() << "Requested settable write access to element "
                       << reactor_model_->element_registry.get(effect_uid).fqn << " which is not a port.";
    return nullptr;
  }

  auto it = settable_effects_.find(effect_uid);
  if (it == settable_effects_.end()) {
    auto res = settable_effects_.try_emplace(effect_uid, effect_uid, scheduler_);
    util::assert_(res.second);
    it = res.first;
  }

  return &it->second;
}

auto DefaultRuntimeImpl::get_schedulable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
    -> SchedulableEffect* {
  if (!is_valid_effect(reaction_uid, effect_uid)) {
    util::log::error() << "Requested write access to element " << reactor_model_->element_registry.get(effect_uid).fqn
                       << " which is not a registered effect of reaction "
                       << reactor_model_->element_registry.get(reaction_uid).fqn;
    return nullptr;
  }

  if (const auto& elem = reactor_model_->element_registry.get(effect_uid);
      !std::holds_alternative<core::ProgrammableTimerTag>(elem.type)) {
    util::log::error() << "Requested schedulable write access to element "
                       << reactor_model_->element_registry.get(effect_uid).fqn << " which is not a programmable timer.";
    return nullptr;
  }

  auto it = schedulable_effects_.find(effect_uid);
  if (it == schedulable_effects_.end()) {
    auto res = schedulable_effects_.try_emplace(effect_uid, effect_uid, scheduler_);
    util::assert_(res.second);
    it = res.first;
  }

  return &it->second;
}

auto DefaultRuntimeImpl::get_shutdown_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
    -> ShutdownEffect* {
  if (!is_valid_effect(reaction_uid, effect_uid)) {
    util::log::error() << "Requested write access to element " << reactor_model_->element_registry.get(effect_uid).fqn
                       << " which is not a registered effect of reaction "
                       << reactor_model_->element_registry.get(reaction_uid).fqn;
    return nullptr;
  }

  if (const auto& elem = reactor_model_->element_registry.get(effect_uid);
      !std::holds_alternative<core::ShutdownTag>(elem.type)) {
    util::log::error() << "Requested shutdown access to element "
                       << reactor_model_->element_registry.get(effect_uid).fqn << " which is not a shutdown element.";
    return nullptr;
  }

  return &shutdown_effect_;
}

auto DefaultRuntimeImpl::get_physical_event(std::uint64_t physical_event_uid) noexcept -> PhysicalEvent* {
  // The map holds every physical event since initialize and is never mutated
  // afterwards, which makes concurrent lookups from external threads safe.
  auto it = physical_events_.find(physical_event_uid);
  if (it != physical_events_.end()) {
    return &it->second;
  }

  // Population is exhaustive, so a miss means the uid names no physical
  // event. Callers may pass arbitrary uids, so resolve the FQN only for
  // registered ones; `get` has no miss path.
  if (reactor_model_->element_registry.contains(physical_event_uid)) {
    util::log::error() << "Requested write access to element "
                       << reactor_model_->element_registry.get(physical_event_uid).fqn
                       << " which is not a physical event.";
  } else {
    util::log::error() << "Requested write access to unknown element uid " << physical_event_uid << ".";
  }
  return nullptr;
}

auto DefaultRuntimeImpl::is_valid_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept
    -> bool {
  auto it = runtime_model_->triggers.find(trigger_uid);
  return it != runtime_model_->triggers.end() && std::ranges::find(it->second.triggered_reaction_uids, reaction_uid) !=
                                                     it->second.triggered_reaction_uids.end();
}

auto DefaultRuntimeImpl::is_valid_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) const noexcept -> bool {
  auto effects_view = reactor_model_->reaction_dependency_registry.get_effects(reaction_uid);
  return std::ranges::find(effects_view, effect_uid) != effects_view.end();
}

} // namespace xronos::runtime::default_::detail
