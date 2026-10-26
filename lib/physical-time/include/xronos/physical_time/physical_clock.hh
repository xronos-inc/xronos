// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_PHYSICAL_TIME_PHYSICAL_CLOCK_HH
#define XRONOS_PHYSICAL_TIME_PHYSICAL_CLOCK_HH

#include "xronos/core/time.hh"
#include "xronos/physical_time/interrupt.hh"

namespace xronos::physical_time {

// The clock a runtime reads for physical time. `wait_until` takes an
// interrupt token so a caller can end a wait early without owning a
// condition variable of its own. A test substitutes a clock it controls to
// drive a run loop without waiting for real time to pass.
class PhysicalClock {
public:
  virtual ~PhysicalClock() = default;

  [[nodiscard]] virtual auto now() const noexcept -> core::TimePoint = 0;

  // Blocks until the clock reaches `time_point` or `interrupt` is
  // interrupted. Returns true once the time point has passed and false when
  // the interrupt came first.
  [[nodiscard]] virtual auto wait_until(core::TimePoint time_point, const InterruptToken& interrupt) const noexcept
      -> bool = 0;
};

// The operating system's real-time clock.
class SystemClock final : public PhysicalClock {
public:
  [[nodiscard]] auto now() const noexcept -> core::TimePoint final;
  [[nodiscard]] auto wait_until(core::TimePoint time_point, const InterruptToken& interrupt) const noexcept
      -> bool final;
};

} // namespace xronos::physical_time

#endif // XRONOS_PHYSICAL_TIME_PHYSICAL_CLOCK_HH
