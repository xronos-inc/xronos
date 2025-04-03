// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the inputs and outputs of reactors.
 */

#ifndef XRONOS_SDK_PORT_HH
#define XRONOS_SDK_PORT_HH

#include <memory>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

#include "xronos/runtime/port.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::sdk {

/**
 * @brief Base class for inputs and outputs of reactors.
 *
 * @details This should not be inherited from or instantiated directly by
 * application code. Instead, use `InputPort` or `OutputPort`.
 * @tparam T The type of values carried by the port.
 */
template <class T> class Port : public Element, public EventSource<T> {
protected:
  Port(std::unique_ptr<runtime::Port<T>> runtime_instance, ReactorContext context)
      : Element(std::move(runtime_instance), context) {}

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::get_runtime_instance<runtime::Port<T>>(*this).is_present();
  }
  [[nodiscard]] auto get() const noexcept -> const ImmutableValuePtr<T>& final {
    return detail::get_runtime_instance<runtime::Port<T>>(*this).get();
  }
  void set(const ImmutableValuePtr<T>& value) noexcept {
    detail::get_runtime_instance<runtime::Port<T>>(*this).set(value);
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_trigger(&detail::get_runtime_instance<runtime::BasePort>(*this));
  }

  void register_as_source_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_dependency(&detail::get_runtime_instance<runtime::BasePort>(*this));
  }

  friend BaseReaction;
};

/**
 * @brief Base class for ports that do not communicate values.
 *
 * @details This should not be inherited from or instantiated directly by application code.
 * @details Ports of this type serve only to trigger activity or to signal
 * presence or absence of an event.
 */
template <> class Port<void> : public Element, public EventSource<void> {
protected:
  Port(std::unique_ptr<runtime::Port<void>> runtime_instance, ReactorContext context)
      : Element(std::move(runtime_instance), context) {}

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::get_runtime_instance<runtime::Port<void>>(*this).is_present();
  }
  void set() noexcept { detail::get_runtime_instance<runtime::Port<void>>(*this).set(); }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_trigger(&detail::get_runtime_instance<runtime::BasePort>(*this));
  }

  void register_as_source_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_dependency(&detail::get_runtime_instance<runtime::BasePort>(*this));
  }

  friend BaseReaction;
};

/**
 * @brief A port that receives values from other reactors.
 *
 * @details The input port does not provide direct access to received values. A
 * reaction may declare an input port as a `BaseReaction::Trigger` or
 * `BaseReaction::Source` to read its value.
 * @tparam T The type of values carried by the port.
 */
template <class T> class InputPort final : public Port<T> {
public:
  /**
   * @brief Construct a new `InputPort` object.
   *
   * @param name The name of the port.
   * @param context The current reactor's initialization context, which can
   * be obtained using the `Reactor::context` method.
   */
  InputPort(std::string_view name, ReactorContext context)
      : Port<T>{std::make_unique<runtime::Input<T>>(name, detail::get_reactor_instance(context)), context} {}
};
/**
 * @brief A port that sends values to other reactors.
 *
 * The output port does not provide direct access for writing values. A reaction
 * may declare a `BaseReaction::PortEffect` to send a value.
 * @tparam T The type of values carried by the port.
 */
template <class T> class OutputPort final : public Port<T> {
public:
  /**
   * @brief Construct a new `OutputPort` object.
   *
   * @param name The name of the port.
   * @param context The current reactor's initialization context, which can
   * be obtained using the `Reactor::context` method.
   */
  OutputPort(std::string_view name, ReactorContext context)
      : Port<T>{std::make_unique<runtime::Output<T>>(name, detail::get_reactor_instance(context)), context} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PORT_HH
