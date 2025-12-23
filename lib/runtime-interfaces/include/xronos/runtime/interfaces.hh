// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_INTERFACES_HH
#define XRONOS_RUNTIME_INTERFACES_HH

#include <any>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string_view>

#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"

namespace xronos::runtime {

struct ValidationError : public std::runtime_error {
  explicit ValidationError(std::string_view msg)
      : std::runtime_error{std::string{msg}} {}
};

struct GettableTrigger {
  [[nodiscard]] virtual auto get() const noexcept -> std::any = 0;
  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;

protected:
  ~GettableTrigger() = default;
};

struct SettableEffect {
  virtual void set(const std::any& value) noexcept = 0;
  [[nodiscard]] virtual auto get() const noexcept -> std::any = 0;
  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;

protected:
  ~SettableEffect() = default;
};

struct SchedulableEffect {
  virtual void schedule(const std::any& value, core::Duration delay) noexcept = 0;

protected:
  ~SchedulableEffect() = default;
};

struct ShutdownEffect {
  virtual void trigger_shutdown() noexcept = 0;

protected:
  ~ShutdownEffect() = default;
};

struct ExternalTrigger {
  virtual void trigger(const std::any& value) noexcept = 0;

protected:
  ~ExternalTrigger() = default;
};

struct TimeAccess {
  [[nodiscard]] virtual auto get_timestamp() const noexcept -> core::TimePoint = 0;
  [[nodiscard]] virtual auto get_microstep() const noexcept -> std::uint32_t = 0;
  [[nodiscard]] virtual auto get_start_timestamp() const noexcept -> core::TimePoint = 0;

protected:
  ~TimeAccess() = default;
};

struct ProgramHandle {
  [[nodiscard]] virtual auto get_trigger(std::uint64_t reaction_uid, std::uint64_t trigger_uid) const noexcept
      -> const GettableTrigger* = 0;
  [[nodiscard]] virtual auto get_settable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> SettableEffect* = 0;
  [[nodiscard]] virtual auto get_schedulable_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> SchedulableEffect* = 0;
  [[nodiscard]] virtual auto get_shutdown_effect(std::uint64_t reaction_uid, std::uint64_t effect_uid) noexcept
      -> ShutdownEffect* = 0;
  [[nodiscard]] virtual auto get_time_access(std::uint64_t reactor_uid) const noexcept -> const TimeAccess* = 0;
  [[nodiscard]] virtual auto get_external_trigger(std::uint64_t external_trigger_uid) noexcept -> ExternalTrigger* = 0;

  virtual void execute() = 0;

  virtual ~ProgramHandle() = default;
};

struct ExecutionProperties {
  core::Duration timeout{core::Duration::max()};
  uint32_t num_workers{0};
  bool fast_mode{false};
};

struct Runtime {
  [[nodiscard]] virtual auto initialize_reactor_program(const core::ReactorModel& model,
                                                        const ExecutionProperties& properties)
      -> std::unique_ptr<ProgramHandle> = 0;
  virtual ~Runtime() = default;
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_INTERFACES_HH
