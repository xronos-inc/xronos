// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/reaction.hh"

#include <any>
#include <cstdint>
#include <memory>
#include <utility>
#include <variant>

#include "impl/xronos/sdk/detail/element.hh"
#include "impl/xronos/sdk/detail/program_context.hh"
#include "xronos/core/element.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/shutdown.hh"
#include "xronos/sdk/time.hh"
#include "xronos/util/assert.hh"

namespace xronos::sdk {

BaseReaction::BaseReaction(const ReactionProperties& properties)
    : Element{detail::register_element(
                  properties.name_,
                  core::ReactionTag{std::make_unique<core::ReactionProperties>(core::ReactionProperties{
                      .handler =
                          [this]() {
                            if (program_context()->runtime_program_handle == nullptr) {
                              throw ValidationError{"Reaction called without a valid execution context."};
                            }
                            auto scope =
                                program_context()->telemetry_backend->reaction_span_logger().record_reaction_span(
                                    uid(), *program_context()->runtime_program_handle);
                            handler();
                          },
                      .position = properties.position_})},
                  properties.context_),
              properties.context_} {}

BaseReaction::TriggerImpl::TriggerImpl(std::uint64_t trigger_uid, const ReactionContext& context)
    : trigger_uid_{trigger_uid}
    , reaction_uid_{context.reaction_instance().uid()}
    , program_context_{*context.reaction_instance().program_context()} {
  context.reaction_instance().program_context()->model.reaction_dependency_registry.register_reaction_trigger(
      reaction_uid_, trigger_uid_);
}

auto BaseReaction::TriggerImpl::get_impl() const noexcept -> const runtime::GettableTrigger* {
  if (impl_ == nullptr) {
    if (program_context_.get().runtime_program_handle != nullptr) {
      impl_ = program_context_.get().runtime_program_handle->get_trigger(reaction_uid_, trigger_uid_);
      util::assert_(impl_ != nullptr);
    }
  }

  return impl_;
}

auto BaseReaction::TriggerImpl::get() const noexcept -> std::any {
  if (const auto* impl = get_impl(); impl != nullptr) {
    return impl->get();
  }
  return std::monostate{};
}

auto BaseReaction::TriggerImpl::is_present() const noexcept -> bool {
  if (const auto* impl = get_impl(); impl != nullptr) {
    return impl->is_present();
  }
  return false;
}

BaseReaction::PortEffectImpl::PortEffectImpl(std::uint64_t effect_uid, const ReactionContext& context)
    : effect_uid_{effect_uid}
    , reaction_uid_{context.reaction_instance().uid()}
    , program_context_{*context.reaction_instance().program_context()} {
  context.reaction_instance().program_context()->model.reaction_dependency_registry.register_reaction_effect(
      reaction_uid_, effect_uid_);
}

auto BaseReaction::PortEffectImpl::get_impl() noexcept -> runtime::SettableEffect* {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  return const_cast<runtime::SettableEffect*>(std::as_const(*this).get_impl());
}

auto BaseReaction::PortEffectImpl::get_impl() const noexcept -> const runtime::SettableEffect* {
  if (impl_ == nullptr) {
    if (program_context_.get().runtime_program_handle != nullptr) {
      impl_ = program_context_.get().runtime_program_handle->get_settable_effect(reaction_uid_, effect_uid_);
      util::assert_(impl_ != nullptr);
    }
  }

  return impl_;
}

void BaseReaction::PortEffectImpl::set(const std::any& value) noexcept {
  if (auto* impl = get_impl(); impl != nullptr) {
    impl->set(value);
  }
}

auto BaseReaction::PortEffectImpl::get() const noexcept -> std::any {
  if (const auto* impl = get_impl(); impl != nullptr) {
    return impl->get();
  }
  return std::monostate{};
}

auto BaseReaction::PortEffectImpl::is_present() const noexcept -> bool {
  if (const auto* impl = get_impl(); impl != nullptr) {
    return impl->is_present();
  }
  return false;
}

BaseReaction::ProgrammableTimerEffectImpl::ProgrammableTimerEffectImpl(std::uint64_t effect_uid,
                                                                       const ReactionContext& context)
    : effect_uid_{effect_uid}
    , reaction_uid_{context.reaction_instance().uid()}
    , program_context_{*context.reaction_instance().program_context()} {
  context.reaction_instance().program_context()->model.reaction_dependency_registry.register_reaction_effect(
      reaction_uid_, effect_uid_);
}

auto BaseReaction::ProgrammableTimerEffectImpl::get_impl() noexcept -> runtime::SchedulableEffect* {
  if (impl_ == nullptr) {
    if (program_context_.get().runtime_program_handle != nullptr) {
      impl_ = program_context_.get().runtime_program_handle->get_schedulable_effect(reaction_uid_, effect_uid_);
      util::assert_(impl_ != nullptr);
    }
  }

  return impl_;
}

void BaseReaction::ProgrammableTimerEffectImpl::schedule(const std::any& value, Duration delay) noexcept {
  if (auto* impl = get_impl(); impl != nullptr) {
    impl->schedule(value, delay);
  }
}

BaseReaction::ShutdownEffect::ShutdownEffect(Shutdown& shutdown, const ReactionContext& context)
    : effect_uid_{shutdown.uid()}
    , reaction_uid_{context.reaction_instance().uid()}
    , program_context_{*context.reaction_instance().program_context()} {
  context.reaction_instance().program_context()->model.reaction_dependency_registry.register_reaction_effect(
      reaction_uid_, effect_uid_);
}

auto BaseReaction::ShutdownEffect::get_impl() noexcept -> runtime::ShutdownEffect* {
  if (impl_ == nullptr) {
    if (program_context_.get().runtime_program_handle != nullptr) {
      impl_ = program_context_.get().runtime_program_handle->get_shutdown_effect(reaction_uid_, effect_uid_);
      util::assert_(impl_ != nullptr);
    }
  }

  return impl_;
}

void BaseReaction::ShutdownEffect::trigger_shutdown() noexcept {
  if (auto* impl = get_impl(); impl != nullptr) {
    impl->trigger_shutdown();
  }
}

} // namespace xronos::sdk
