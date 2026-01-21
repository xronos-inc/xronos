// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_LOGICAL_TIME_TAG_HH
#define XRONOS_LOGICAL_TIME_TAG_HH

#include <cstdint>
#include <limits>
#include <ostream>

#include "xronos/core/time.hh"

namespace xronos::logical_time {

class Tag {
public:
  Tag() = default;
  Tag(const core::TimePoint timestamp, std::uint32_t microstep = 0)
      : timestamp_{timestamp}
      , microstep_{microstep} {}

  [[nodiscard]] auto timestamp() const noexcept { return timestamp_; }
  [[nodiscard]] auto microstep() const noexcept { return microstep_; }

  [[nodiscard]] auto operator<=>(const Tag&) const = default;

  [[nodiscard]] auto operator+(core::Duration delay) const -> Tag;
  [[nodiscard]] auto operator-(core::Duration delay) const -> Tag;

  [[nodiscard]] auto increment_microstep() const -> Tag;
  [[nodiscard]] auto decrement_microstep() const -> Tag;

  [[nodiscard]] static auto max() noexcept -> Tag {
    return {core::TimePoint::max(), std::numeric_limits<std::uint32_t>::max()};
  }

private:
  core::TimePoint timestamp_{};
  std::uint32_t microstep_{0};
};

auto operator<<(std::ostream& ostream, const Tag& tag) -> std::ostream&;

} // namespace xronos::logical_time

#endif // XRONOS_LOGICAL_TIME_TAG_HH
