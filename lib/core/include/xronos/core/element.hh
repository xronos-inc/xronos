// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_ELEMENT_HH
#define XRONOS_CORE_ELEMENT_HH

#include <any>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "xronos/core/time.hh"

namespace xronos::core {

using ElementID = std::uint64_t;

struct ReactionProperties {
  std::function<void()> handler;
  std::uint32_t position{};
  std::optional<Duration> deadline;
};

struct MetricProperties {
  std::string description;
  std::string unit;
};

struct PeriodicTimerProperties {
  Duration offset;
  Duration period;
};

struct PortProperties {
  std::function<std::vector<std::byte>(const std::any&)> serializer{nullptr};
  std::function<std::any(std::span<const std::byte>)> deserializer{nullptr};
};

// These <name>Tag classes are used similarly to enum values. They identify the
// concrete element type. Some tags are annotated with additional type specific
// properties. We use the indirection of an std::unique_ptr here to ensure that
// tag types remain small (8 bytes on most 64-bit machines).
struct InputPortTag {
  std::unique_ptr<PortProperties> properties;
};
struct MetricTag {
  std::unique_ptr<MetricProperties> properties;
};
struct OutputPortTag {
  std::unique_ptr<PortProperties> properties;
};
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

auto get_port_properties(const Element& element) -> PortProperties&;

auto element_type_as_string(const ElementType& type) -> std::string_view;

} // namespace xronos::core

#endif // XRONOS_CORE_ELEMENT_HH
