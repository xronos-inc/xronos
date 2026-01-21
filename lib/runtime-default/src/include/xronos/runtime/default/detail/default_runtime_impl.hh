// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DETAIL_DEFAULT_RUNTIME_IMPL_HH
#define XRONOS_RUNTIME_DEFAULT_DETAIL_DEFAULT_RUNTIME_IMPL_HH

#include <any>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/runtime/default/detail/scheduler.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_::detail {

class DefaultTrigger final : public GettableTrigger {
public:
  DefaultTrigger(std::uint64_t uid, const Scheduler& scheduler)
      : event_source_{scheduler.get_event_source(uid)} {}

  [[nodiscard]] auto get() const noexcept -> std::any final { return event_source_.get().get(); }
  [[nodiscard]] auto is_present() const noexcept -> bool final { return event_source_.get().is_present(); }

private:
  std::reference_wrapper<const EventSource> event_source_;
};

class DefaultSettableEffect final : public SettableEffect {
public:
  DefaultSettableEffect(std::uint64_t uid, Scheduler& scheduler)
      : event_source_{scheduler.get_event_source(uid)}
      , scheduler_{scheduler}
      , uid_{uid} {}

  void set(const std::any& value) noexcept final { scheduler_.get().set_port(uid_, value); }
  [[nodiscard]] auto get() const noexcept -> std::any final { return event_source_.get().get(); }
  [[nodiscard]] auto is_present() const noexcept -> bool final { return event_source_.get().is_present(); }

private:
  std::reference_wrapper<const EventSource> event_source_;
  std::reference_wrapper<Scheduler> scheduler_;
  std::uint64_t uid_;
};

class DefaultSchedulableEffect final : public SchedulableEffect {
public:
  DefaultSchedulableEffect(std::uint64_t uid, Scheduler& scheduler)
      : scheduler_{scheduler}
      , uid_{uid} {}

  void schedule(const std::any& value, core::Duration delay) noexcept final {
    scheduler_.get().schedule_event(uid_, value, delay);
  }

private:
  std::reference_wrapper<Scheduler> scheduler_;
  std::uint64_t uid_;
};

class DefaultExternalTrigger final : public ExternalTrigger {
public:
  DefaultExternalTrigger(std::uint64_t uid, Scheduler& scheduler)
      : uid_{uid}
      , scheduler_{scheduler} {}

  void trigger(const std::any& value) noexcept final { scheduler_.get().trigger_external_event(uid_, value); }

private:
  std::uint64_t uid_;
  std::reference_wrapper<Scheduler> scheduler_;
};

class DefaultShutdownEffect final : public ShutdownEffect {
public:
  DefaultShutdownEffect(Scheduler& scheduler)
      : scheduler_{scheduler} {}

  void trigger_shutdown() noexcept final { scheduler_.get().trigger_shutdown(); };

private:
  std::reference_wrapper<Scheduler> scheduler_;
};

class DefaultTimeAccess final : public TimeAccess {
public:
  DefaultTimeAccess(const Scheduler& scheduler)
      : scheduler_{scheduler} {}

  [[nodiscard]] auto get_timestamp() const noexcept -> core::TimePoint final {
    return scheduler_.get().get_current_tag().timestamp();
  }
  [[nodiscard]] auto get_microstep() const noexcept -> std::uint32_t final {
    return scheduler_.get().get_current_tag().microstep();
  }
  [[nodiscard]] auto get_start_timestamp() const noexcept -> core::TimePoint final {
    return scheduler_.get().get_start_tag().timestamp();
  }

private:
  std::reference_wrapper<const Scheduler> scheduler_;
};

class DefaultRuntimeImpl final : public ProgramHandle {
public:
  DefaultRuntimeImpl(const ExecutionProperties& properties)
      : scheduler_{properties} {}
  DefaultRuntimeImpl(const DefaultRuntimeImpl&) = delete;
  DefaultRuntimeImpl(DefaultRuntimeImpl&&) = delete;
  auto operator=(const DefaultRuntimeImpl&) = delete;
  auto operator=(DefaultRuntimeImpl&&) -> DefaultRuntimeImpl& = delete;
  ~DefaultRuntimeImpl() final = default;

  void initialize(const core::ReactorModel& model);
  void execute() final { scheduler_.execute(); }

  [[nodiscard]] auto get_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept
      -> const GettableTrigger* final;
  [[nodiscard]] auto get_settable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> SettableEffect* final;
  [[nodiscard]] auto get_schedulable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> SchedulableEffect* final;
  [[nodiscard]] auto get_shutdown_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> ShutdownEffect* final;
  [[nodiscard]] auto get_external_trigger(std::uint64_t external_trigger_uid) noexcept -> ExternalTrigger* final;
  [[nodiscard]] auto get_time_access([[maybe_unused]] std::uint64_t reactor_uid) const noexcept
      -> const TimeAccess* final {
    return &time_access_;
  }

private:
  std::unique_ptr<RuntimeModel> runtime_model_{nullptr};
  const core::ReactorModel* reactor_model_{nullptr};

  mutable std::unordered_map<std::uint64_t, DefaultTrigger> triggers_;
  std::unordered_map<std::uint64_t, DefaultSettableEffect> settable_effects_;
  std::unordered_map<std::uint64_t, DefaultSchedulableEffect> schedulable_effects_;
  std::unordered_map<std::uint64_t, DefaultExternalTrigger> external_triggers_;

  Scheduler scheduler_;
  DefaultTimeAccess time_access_{scheduler_};
  DefaultShutdownEffect shutdown_effect_{scheduler_};

  auto is_valid_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept -> bool;
  auto is_valid_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) const noexcept -> bool;
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_DEFAULT_RUNTIME_IMPL_HH
