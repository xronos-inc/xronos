// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/physical_event.hh"

#include <any>
#include <cstdint>
#include <string_view>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "xronos/core/element.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/sdk/context.hh"
#include "xronos/util/assert.hh"

namespace xronos::sdk::detail {

using CA = ContextAccess;

auto register_physical_event(std::string_view name, const ReactorContext& context) -> const core::Element& {
  return CA::get_program_context(context)->model.element_registry.add_new_element(name, core::PhysicalEventTag{},
                                                                                  CA::get_parent_uid(context));
}

PhysicalEventImpl::PhysicalEventImpl(std::uint64_t uid, const ReactorContext& context)
    : uid_{uid}
    , program_context_{*CA::get_program_context(context)} {}

void PhysicalEventImpl::trigger(const std::any& value) {
  if (auto* impl = get_impl(); impl != nullptr) {
    impl->trigger(value);
  }
}

auto PhysicalEventImpl::get_impl() noexcept -> runtime::ExternalTrigger* {
  if (impl_ == nullptr) {
    if (program_context_.get().runtime_program_handle != nullptr) {
      impl_ = program_context_.get().runtime_program_handle->get_external_trigger(uid_);
      util::assert_(impl_ != nullptr);
    }
  }

  return impl_;
}

} // namespace xronos::sdk::detail
