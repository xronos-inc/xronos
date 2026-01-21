// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/element.hh"

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

#include "fmt/format.h"
#include "impl/xronos/sdk/detail/context_access.hh"
#include "impl/xronos/sdk/detail/element.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/source_location/source_location.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::sdk {

Element::Element(const core::Element& core_element, const Context& context)
    : core_element_{core_element}
    , program_context_{detail::ContextAccess::get_program_context(context)} {
  auto source_location_view = detail::ContextAccess::get_source_location(context);
  program_context()->source_location_registry.add_source_location(
      uid(), source_location::SourceLocation{
                 .file = std::string{source_location_view.file},
                 .function = std::string{source_location_view.function},
                 .start_line = source_location_view.start_line,
                 .end_line = source_location_view.end_line,
                 .start_column = source_location_view.start_column,
                 .end_column = source_location_view.end_column,
             });
}

auto Element::name() const noexcept -> const std::string& { return core_element().name; }
auto Element::fqn() const noexcept -> const std::string& { return core_element().fqn; }
auto Element::uid() const noexcept -> std::uint64_t { return core_element().uid; }

auto Element::add_attribute(std::string_view key, const AttributeValue& value) noexcept -> bool {
  return program_context()->attribute_manager.add_attribute(uid(), key, value);
}

namespace detail {

auto register_element(std::string_view name, core::ElementType type, const Context& context) -> const core::Element& {
  auto element = ContextAccess::get_program_context(context)->model.element_registry.add_new_element(
      name, std::move(type), ContextAccess::get_parent_uid(context));

  if (!element.has_value()) {
    auto source_location = ContextAccess::get_source_location(context);

    throw ::xronos::sdk::DuplicateNameError(
        fmt::format("{}:{}: {}", source_location.file, source_location.start_line, element.error()));
  }

  return *element;
}

} // namespace detail

} // namespace xronos::sdk
