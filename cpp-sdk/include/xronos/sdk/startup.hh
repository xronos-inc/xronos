// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_STARTUP_HH
#define XRONOS_SDK_STARTUP_HH

#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

/**
 * A reactor element that emits an event when the program starts up.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger".
 */
class Startup final : public Element, public EventSource<void> {
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
  Startup(std::string_view name, ReactorContext context);

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final;

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_STARTUP_HH
