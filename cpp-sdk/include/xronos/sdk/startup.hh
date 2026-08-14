// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_STARTUP_HH
#define XRONOS_SDK_STARTUP_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk {

namespace detail {

inline auto register_startup(std::string_view name, const ReactorContext& context) -> std::uint64_t {
  return register_with_location(context, [&]() -> std::uint64_t {
    return get_backend(context).register_startup(std::string{name}, ContextAccess::get_parent_uid(context));
  });
}

} // namespace detail

/**
 * A reactor element that emits an event when the program starts up.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger".
 */
class Startup final : public Element {
public:
  /**
   * Constructor.
   *
   * Typically user code should not need to instantiate this directly, as each
   * reactor provides an instance via Reactor::startup().
   *
   * @param name The name of the element.
   * @param context The containing reactor's context.
   */
  Startup(std::string_view name, ReactorContext context)
      : Element{detail::register_startup(name, context), name, context} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_STARTUP_HH
