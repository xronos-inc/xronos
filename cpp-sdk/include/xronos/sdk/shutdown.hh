// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_SHUTDOWN_HH
#define XRONOS_SDK_SHUTDOWN_HH

#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

/**
 * A reactor element that emits an event right before the program shuts down.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger".
 */
class Shutdown final : public Element {
public:
  /**
   * Constructor.
   *
   * Typically user code should not need to instantiate this directly, as each
   * reactor provides an instance via Reactor::shutdown().
   *
   * @param name The name of the element.
   * @param context The containing reactor's context.
   */
  Shutdown(std::string_view name, ReactorContext context);
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_SHUTDOWN_HH
