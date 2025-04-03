// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/logical_time.hh"

#include "xronos/runtime/assert.hh"
#include <limits>

namespace xronos::runtime {

auto operator==(const Tag& lhs, const Tag& rhs) noexcept -> bool {
  return lhs.time_point() == rhs.time_point() && lhs.micro_step() == rhs.micro_step();
}

auto operator<(const Tag& lhs, const Tag& rhs) noexcept -> bool {
  return lhs.time_point() < rhs.time_point() ||
         (lhs.time_point() == rhs.time_point() && lhs.micro_step() < rhs.micro_step());
}

auto Tag::from_physical_time(TimePoint time_point) noexcept -> Tag { return {time_point, 0}; }

auto Tag::from_logical_time(const LogicalTime& logical_time) noexcept -> Tag {
  return {logical_time.time_point(), logical_time.micro_step()};
}

auto Tag::delay(Duration offset) const noexcept -> Tag {
  if (offset == Duration::zero()) {
    validate(this->micro_step_ != std::numeric_limits<mstep_t>::max(), "Microstep overflow detected!");
    return {this->time_point_, this->micro_step_ + 1};
  }
  return {this->time_point_ + offset, 0};
}

auto Tag::subtract(Duration offset) const noexcept -> Tag {
  if (offset == Duration::zero()) {
    return decrement();
  }
  return {time_point_ - offset, std::numeric_limits<mstep_t>::max()};
}

auto Tag::decrement() const noexcept -> Tag {
  if (micro_step_ == 0) {
    return {time_point_ - Duration{1}, std::numeric_limits<mstep_t>::max()};
  }
  return {time_point_, micro_step_ - 1};
}

void LogicalTime::advance_to(const Tag& tag) {
  reactor_assert(*this < tag);
  time_point_ = tag.time_point();
  micro_step_ = tag.micro_step();
}

void LogicalTime::advance_to(const LogicalTime& time) {
  reactor_assert(*this < Tag::from_logical_time(time));
  time_point_ = time.time_point();
  micro_step_ = time.micro_step();
}

auto operator==(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool {
  return logical_time.time_point() == tag.time_point() && logical_time.micro_step() == tag.micro_step();
}

auto operator<(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool {
  return logical_time.time_point() < tag.time_point() ||
         (logical_time.time_point() == tag.time_point() && logical_time.micro_step() < tag.micro_step());
}

auto operator>(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool {
  return logical_time.time_point() > tag.time_point() ||
         (logical_time.time_point() == tag.time_point() && logical_time.micro_step() > tag.micro_step());
}

auto operator<<(std::ostream& os, const Tag& tag) -> std::ostream& {
  return os << '[' << tag.time_point() << ", " << tag.micro_step() << ']';
}
auto operator<<(std::ostream& os, const LogicalTime& tag) -> std::ostream& {
  return os << '[' << tag.time_point() << ", " << tag.micro_step() << ']';
}

} // namespace xronos::runtime
