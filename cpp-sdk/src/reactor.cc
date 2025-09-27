// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <source_location>
#include <string_view>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/startup.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

using CA = detail::ContextAccess;

Reactor::Reactor(std::string_view name, const Context& parent_context)
    : Element{CA::get_program_context(parent_context)
                  ->model.element_registry.add_new_element(name, core::ReactorTag{},
                                                           CA::get_parent_uid(parent_context)),
              parent_context}
    , startup_{"startup", this->context()}
    , shutdown_{"shutdown", this->context()} {
  program_context()->assemble_callbacks.emplace_back([this]() { assemble(); });
}

[[nodiscard]] auto Reactor::context(std::source_location source_location) noexcept -> ReactorContext {
  return context(detail::SourceLocationView::from_std(source_location));
}

[[nodiscard]] auto Reactor::context(detail::SourceLocationView source_location) noexcept -> ReactorContext {
  return CA::create_reactor_context(program_context(), core_element().uid, source_location);
}

auto Reactor::get_time() const noexcept -> TimePoint {
  if (program_context()->runtime_program_handle == nullptr) {
    return TimePoint::min();
  }
  return program_context()->runtime_program_handle->get_time_access(uid())->get_timestamp();
}

auto Reactor::get_lag() const noexcept -> Duration { return std::chrono::system_clock::now() - get_time(); }

auto Reactor::get_time_since_startup() const noexcept -> Duration {
  if (program_context()->runtime_program_handle == nullptr) {
    return Duration::zero();
  }
  const auto* time_access = program_context()->runtime_program_handle->get_time_access(uid());
  return time_access->get_timestamp() - time_access->get_start_timestamp();
}

} // namespace xronos::sdk
