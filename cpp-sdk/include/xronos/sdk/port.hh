// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PORT_HH
#define XRONOS_SDK_PORT_HH

#include <any>
#include <concepts>
#include <cstddef>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

namespace detail {

auto register_input_port(std::string_view name, const ReactorContext& context) -> const core::Element&;
auto register_output_port(std::string_view name, const ReactorContext& context) -> const core::Element&;

void set_input_serializer(const core::Element& element,
                          const std::function<std::vector<std::byte>(const std::any&)>& serializer);
void set_input_deserializer(const core::Element& element,
                            const std::function<std::any(std::span<const std::byte>)>& deserializer);
void set_output_serializer(const core::Element& element,
                           const std::function<std::vector<std::byte>(const std::any&)>& serializer);
void set_output_deserializer(const core::Element& element,
                             const std::function<std::any(std::span<const std::byte>)>& deserializer);

} // namespace detail

/**
 * Constraints for a serializer that can be used on ports.
 */
template <template <class> class S, class T>
concept IsSerializer = std::is_same_v<T, void> || (requires(T value) {
  { S<T>::serialize(value) } -> std::same_as<std::vector<std::byte>>;
} && requires(std::span<const std::byte> data) {
  { S<T>::deserialize(data) } -> std::same_as<T>;
});

/**
 * Tag type that fulfills the IsSerializer constraints and that is used to
 * indicate no serialization.
 */
template <class T> struct NoSerializer {
  static auto serialize(const T& value) -> std::vector<std::byte>;
  static auto deserialize(std::span<const std::byte> data) -> T;
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
template <class T, template <class> class Serializer = NoSerializer>
  requires IsSerializer<Serializer, T>
class InputPort final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  InputPort(std::string_view name, const ReactorContext& context)
      : Element{detail::register_input_port(name, context), context} {
    if constexpr (std::is_same_v<void, T>) {
      detail::set_input_serializer(core_element(),
                                   []([[maybe_unused]] const std::any&) { return std::vector<std::byte>{}; });
      detail::set_input_deserializer(
          core_element(), []([[maybe_unused]] std::span<const std::byte> data) { return std::any{std::monostate{}}; });
    } else if constexpr (!std::is_same_v<NoSerializer<T>, Serializer<T>>) {
      detail::set_input_serializer(core_element(), [](const std::any& value) {
        return Serializer<T>::serialize(*std::any_cast<ImmutableValuePtr<T>>(value));
      });
      detail::set_input_deserializer(core_element(), [](std::span<const std::byte> data) {
        return make_immutable_value<T>(Serializer<T>::deserialize(data));
      });
    }
  }
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
template <class T, template <class> class Serializer = NoSerializer>
  requires IsSerializer<Serializer, T>
class OutputPort final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  OutputPort(std::string_view name, const ReactorContext& context)
      : Element{detail::register_output_port(name, context), context} {
    if constexpr (std::is_same_v<void, T>) {
      detail::set_output_serializer(core_element(),
                                    []([[maybe_unused]] const std::any&) { return std::vector<std::byte>{}; });
      detail::set_output_deserializer(
          core_element(), []([[maybe_unused]] std::span<const std::byte> data) { return std::any{std::monostate{}}; });
    } else if constexpr (!std::is_same_v<NoSerializer<T>, Serializer<T>>) {
      detail::set_output_serializer(core_element(), [](const std::any& value) {
        return Serializer<T>::serialize(*std::any_cast<ImmutableValuePtr<T>>(value));
      });
      detail::set_output_deserializer(core_element(), [](std::span<const std::byte> data) {
        return make_immutable_value<T>(Serializer<T>::deserialize(data));
      });
    }
  }
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PORT_HH
