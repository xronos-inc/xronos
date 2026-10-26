// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_PHYSICAL_TIME_INTERRUPT_HH
#define XRONOS_PHYSICAL_TIME_INTERRUPT_HH

#include <stop_token>
#include <utility>

namespace xronos::physical_time {

// The interrupt side of a wait on a PhysicalClock. A wait takes a token, and
// interrupt() on the token's source ends the wait at once, from any thread.
// An interrupt is never reset: once a source is interrupted, every later wait
// on its tokens ends at once. One source therefore serves every wait that a
// single interrupt should end, such as the waits before an abort, and a caller
// whose interrupt applies to one wait creates a source per wait. The pair
// wraps std::stop_source and std::stop_token, whose names read as a shutdown
// request rather than an ended wait.
class InterruptToken {
public:
  // Adopts a std::stop_token, so a std::jthread's body can pass the token
  // it received to a wait.
  explicit InterruptToken(std::stop_token token) noexcept
      : token_{std::move(token)} {}

  [[nodiscard]] auto interrupted() const noexcept -> bool { return token_.stop_requested(); }

  // The wrapped token, for a clock that waits on a std::condition_variable_any.
  [[nodiscard]] auto stop_token() const noexcept -> const std::stop_token& { return token_; }

private:
  std::stop_token token_;
};

class InterruptSource {
public:
  void interrupt() noexcept { source_.request_stop(); }

  [[nodiscard]] auto interrupted() const noexcept -> bool { return source_.stop_requested(); }

  [[nodiscard]] auto token() const noexcept -> InterruptToken { return InterruptToken{source_.get_token()}; }

private:
  std::stop_source source_{};
};

} // namespace xronos::physical_time

#endif // XRONOS_PHYSICAL_TIME_INTERRUPT_HH
