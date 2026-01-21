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

auto DefaultRuntimeImpl::get_external_trigger(std::uint64_t external_trigger_uid) noexcept -> ExternalTrigger* {
  if (const auto& elem = reactor_model_->element_registry.get(external_trigger_uid);
      !std::holds_alternative<core::PhysicalEventTag>(elem.type)) {
    util::log::error() << "Requested write access to element "
                       << reactor_model_->element_registry.get(external_trigger_uid).fqn
                       << " which is not a physical event.";
    return nullptr;
  }

  auto it = external_triggers_.find(external_trigger_uid);
  if (it == external_triggers_.end()) {
    auto res = external_triggers_.try_emplace(external_trigger_uid, external_trigger_uid, scheduler_);
    util::assert_(res.second);
    it = res.first;
  }

  return &it->second;
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
