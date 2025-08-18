// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PORT_HH
#define XRONOS_SDK_PORT_HH

#include <any>
#include <string_view>
#include <variant>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

namespace detail::runtime_port {

// Helper functions for accessing the underlying runtime code using pImpl.
[[nodiscard]] auto is_present(const Element& port) noexcept -> bool;
[[nodiscard]] auto get(const Element& port) noexcept -> const std::any&;
void set(Element& port, const std::any& value) noexcept;
void register_as_trigger_of(const Element& port, runtime::Reaction& reaction) noexcept;
auto make_input(std::string_view name, ReactorContext context) -> RuntimeElementPtr;
auto make_output(std::string_view name, ReactorContext context) -> RuntimeElementPtr;
void register_as_effect_of(const Element& port, runtime::Reaction& reaction) noexcept;

} // namespace detail::runtime_port

/**
 * Base class for reactor elements that may send or receive messages.
 *
 * This should not be used directly in application code. Use InputPort or
 * OutputPort instead.
 *
 * @tparam T The type of values carried by the port.
 */
template <class T> class Port : public Element, public EventSource<T> {
private:
  using Element::Element;

  [[nodiscard]] auto is_present() const noexcept -> bool final { return detail::runtime_port::is_present(*this); }
  [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T> final {
    if (!is_present()) {
      return ImmutableValuePtr<T>{nullptr};
    }
    return std::any_cast<ImmutableValuePtr<T>>(detail::runtime_port::get(*this));
  }
  void set(const ImmutableValuePtr<T>& value) noexcept { detail::runtime_port::set(*this, value); }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    detail::runtime_port::register_as_trigger_of(*this, reaction);
  }

  friend BaseReaction;
};

/**
 * Base class for reactor elements that may send or receive messages.
 *
 * This should not be used directly in application code. Use InputPort or
 * OutputPort instead.
 *
 * This is a template specialization of Port for sending and receiving messages
 * without a value.
 */
template <> class Port<void> : public Element, public EventSource<void> {
private:
  using Element::Element;

  [[nodiscard]] auto is_present() const noexcept -> bool final { return detail::runtime_port::is_present(*this); }
  void set() noexcept { detail::runtime_port::set(*this, std::monostate{}); }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    detail::runtime_port::register_as_trigger_of(*this, reaction);
  }

  friend BaseReaction;
};

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
template <class T> class InputPort final : public Port<T> {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  InputPort(std::string_view name, ReactorContext context)
      : Port<T>{detail::runtime_port::make_input(name, context), context} {}
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
template <class T> class OutputPort final : public Port<T> {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  OutputPort(std::string_view name, ReactorContext context)
      : Port<T>{detail::runtime_port::make_output(name, context), context} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PORT_HH
