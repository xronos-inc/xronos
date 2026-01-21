// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_STARTUP_HH
#define XRONOS_SDK_STARTUP_HH

#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

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
  Startup(std::string_view name, ReactorContext context);
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_STARTUP_HH
