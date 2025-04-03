// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `Startup` class.
 */

#ifndef XRONOS_SDK_STARTUP_HH
#define XRONOS_SDK_STARTUP_HH

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"

namespace xronos::sdk {
/**
 * @brief An event that triggers when the program starts executing.
 */
class Startup final : public Element, public EventSource<void> {
public:
  /**
   * @brief Construct a new `Startup` object.
   *
   * @param name The name of the startup event.
   * @param context The current reactor's initialization context, which can
   * be obtained using the `Reactor::context` method.
   */
  Startup(std::string_view name, ReactorContext context);

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final;

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final;
  void register_as_source_of(runtime::Reaction& reaction) const noexcept final;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_STARTUP_HH
