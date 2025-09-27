// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PORT_HH
#define XRONOS_SDK_PORT_HH

#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

namespace detail {

auto register_input_port(std::string_view name, const ReactorContext& context) -> const core::Element&;
auto register_output_port(std::string_view name, const ReactorContext& context) -> const core::Element&;

} // namespace detail

/**
 * A reactor element for receiving messages from other reactors.
 *
 * Input ports can be used as a reaction @ref BaseReaction::Trigger "trigger"
 * and provide an interface for reactors to receive messages from other
 * reactors.
 *
 * Input ports may be connected to other ports so that messages are forwarded
 * automatically (see Environment::connect() and Reactor::connect()).
 *
 * Other reactors may also use input ports as a reaction @ref
 * BaseReaction::PortEffect "effect" allowing an external reaction handler to
 * send messages directly to the port.
 *
 * @tparam T The value type associated with messages.
 */
template <class T> class InputPort final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  InputPort(std::string_view name, const ReactorContext& context)
      : Element{detail::register_input_port(name, context), context} {}
};

/**
 * A reactor element for sending messages to other reactors.
 *
 * Output ports can be used as a reaction @ref BaseReaction::PortEffect "effect"
 * and provide an interface for reactors to send messages to other reactors.
 *
 * Output ports may be connected to other ports so that messages are forwarded
 * automatically (see Environment::connect() and Reactor::connect()).
 *
 * Other reactors may also use output ports as a reaction @ref
 * BaseReaction::Trigger "trigger" allowing an external reaction handler to
 * receive messages directly from the port.
 *
 * @tparam T The value type associated with messages.
 */
template <class T> class OutputPort final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  OutputPort(std::string_view name, const ReactorContext& context)
      : Element{detail::register_output_port(name, context), context} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PORT_HH
