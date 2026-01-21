// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/port.hh"

#include <any>
#include <cstddef>
#include <functional>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include "impl/xronos/sdk/detail/element.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk::detail {

auto register_input_port(std::string_view name, const ReactorContext& context) -> const core::Element& {
  return register_element(name, core::InputPortTag{std::make_unique<core::PortProperties>()}, context);
}

auto register_output_port(std::string_view name, const ReactorContext& context) -> const core::Element& {
  return register_element(name, core::OutputPortTag{std::make_unique<core::PortProperties>()}, context);
}

void set_input_serializer(const core::Element& element,
                          const std::function<std::vector<std::byte>(const std::any&)>& serializer) {
  core::get_properties<core::InputPortTag>(element).serializer = serializer;
}
void set_input_deserializer(const core::Element& element,
                            const std::function<std::any(std::span<const std::byte>)>& deserializer) {
  core::get_properties<core::InputPortTag>(element).deserializer = deserializer;
}
void set_output_serializer(const core::Element& element,
                           const std::function<std::vector<std::byte>(const std::any&)>& serializer) {
  core::get_properties<core::OutputPortTag>(element).serializer = serializer;
}
void set_output_deserializer(const core::Element& element,
                             const std::function<std::any(std::span<const std::byte>)>& deserializer) {
  core::get_properties<core::OutputPortTag>(element).deserializer = deserializer;
}

} // namespace xronos::sdk::detail
