// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_ELEMENT_HH
#define XRONOS_CORE_ELEMENT_HH

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace xronos::core {

using ElementID = std::uint64_t;

using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

struct ReactionProperties {
  std::function<void()> handler;
  std::uint32_t position;
};

struct MetricProperties {
  std::string description;
  std::string unit;
};

struct PeriodicTimerProperties {
  Duration offset;
  Duration period;
};

// These <name>Tag classes are used similarly to enum values. They identify the
// concrete element type. Some tags are annotated with additional type specific
// properties. We use the indirection of an std::unique_ptr here to ensure that
// tag types remain small (8 bytes on most 64-bit machines).
struct InputPortTag {};
struct MetricTag {
  std::unique_ptr<MetricProperties> properties;
};
struct OutputPortTag {};
struct PeriodicTimerTag {
  std::unique_ptr<PeriodicTimerProperties> properties;
};
struct PhysicalEventTag {};
struct ProgrammableTimerTag {};
struct ReactionTag {
  std::unique_ptr<ReactionProperties> properties;
};
struct ReactorTag {};
struct ShutdownTag {};
struct StartupTag {};

using ElementType = std::variant<MetricTag, PeriodicTimerTag, PhysicalEventTag, InputPortTag, OutputPortTag,
                                 ProgrammableTimerTag, ReactionTag, ReactorTag, ShutdownTag, StartupTag>;

struct Element {
  std::string name;
  std::string fqn;
  ElementID uid;
  std::optional<ElementID> parent_uid;
  ElementType type;
};

template <class T>
auto get_properties(const Element& element) noexcept -> auto&
  requires requires(T type) { type.properties; }
{
  return *std::get<T>(element.type).properties;
}

} // namespace xronos::core

#endif // XRONOS_CORE_ELEMENT_HH
