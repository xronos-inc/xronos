// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_INTERFACES_HH
#define XRONOS_RUNTIME_INTERFACES_HH

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "xronos/abi/backend.hh"
#include "xronos/abi/exceptions.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"

namespace xronos::runtime {

struct ValidationError : public abi::ValidationError {
  explicit ValidationError(std::string_view msg)
      : abi::ValidationError{std::string{msg}} {}
};

// The hot-path interfaces a runtime hands out are the ABI's hot-path
// interfaces (see xronos/abi/backend.hh): deriving without adding members
// lets a `ProgramHandle`'s pointers upcast to the ABI types with no adapter
// objects and no extra dispatch. The runtime-facing names remain so runtimes
// and forward declarations (e.g. xronos/sdk/fwd.hh) are unaffected.

struct GettableTrigger : abi::GettableTrigger {
protected:
  ~GettableTrigger() = default;
};

struct SettableEffect : abi::SettableEffect {
protected:
  ~SettableEffect() = default;
};

struct SchedulableEffect : abi::SchedulableEffect {
protected:
  ~SchedulableEffect() = default;
};

struct ShutdownEffect : abi::ShutdownEffect {
protected:
  ~ShutdownEffect() = default;
};

struct ExternalTrigger : abi::ExternalTrigger {
protected:
  ~ExternalTrigger() = default;
};

// Extends the ABI's TimeAccess with accessors that only implementation-side
// consumers need (e.g. the OTEL telemetry backend records the microstep).
// The SDK never reads microsteps, so this stays off the ABI; should a public
// microstep API ever ship, appending it to abi::TimeAccess is a minor bump.
struct TimeAccess : abi::TimeAccess {
  [[nodiscard]] virtual auto get_microstep() const noexcept -> std::uint32_t = 0;

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
