// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/element.hh"

#include <string>
#include <string_view>
#include <utility>

#include "xronos/runtime/reactor_element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::sdk {

Element::Element(detail::RuntimeElementPtr runtime_instance, Context context)
    : runtime_instance_{std::move(runtime_instance)}
    , attribute_manager_{detail::get_attribute_manager(detail::get_environment(context))} {
  detail::store_source_location(context, runtime_instance_->uid(), runtime_instance_->fqn());
}

auto Element::name() const noexcept -> const std::string& { return runtime_instance_->name(); }
auto Element::fqn() const noexcept -> const std::string& { return runtime_instance_->fqn(); }

auto Element::add_attribute(std::string_view key, const AttributeValue& value) noexcept -> bool {
  return attribute_manager_.get().add_attribute(*runtime_instance_, key, value);
}

void detail::RuntimeElementDeleter::operator()(runtime::ReactorElement* element) {
  delete element; // NOLINT cppcoreguidelines-owning-memory
}

} // namespace xronos::sdk
