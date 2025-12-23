// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/logical_time/tag.hh"

#include <chrono>
#include <limits>
#include <ostream>

#include "xronos/core/time.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/time_operators.hh"

namespace xronos::logical_time {

auto Tag::operator+(core::Duration delay) const -> Tag {
  util::assert_(delay >= core::Duration::zero());
  if (delay == core::Duration::zero()) {
    return increment_microstep();
  }

  if (core::Duration::max() - timestamp_.time_since_epoch() < delay) {
    // If the above is true, then timestamp_ + delay will overflow. In this case,
    // we simply return the max tag.
    return Tag::max();
  }

  return {timestamp_ + delay, 0};
}

auto Tag::operator-(core::Duration delay) const -> Tag {
  util::assert_(delay >= core::Duration::zero());
  if (delay == core::Duration::zero()) {
    return decrement_microstep();
  }
  return {timestamp_ - delay, std::numeric_limits<decltype(microstep_)>::max()};
}

auto Tag::increment_microstep() const -> Tag {
  if (microstep_ == std::numeric_limits<decltype(microstep_)>::max()) {
    if (timestamp_ == core::TimePoint::max()) {
      return max();
    }
    return {timestamp_ + std::chrono::nanoseconds{1}, 0};
  }
  return {timestamp_, microstep_ + 1};
}

auto Tag::decrement_microstep() const -> Tag {
  if (microstep_ == 0) {
    return {timestamp_ - std::chrono::nanoseconds{1}, std::numeric_limits<decltype(microstep_)>::max()};
  }
  return {timestamp_, microstep_ - 1};
}

auto operator<<(std::ostream& ostream, const Tag& tag) -> std::ostream& {
  using xronos::util::operator<<;
  return ostream << '[' << tag.timestamp() << ", " << tag.microstep() << ']';
}

} // namespace xronos::logical_time
