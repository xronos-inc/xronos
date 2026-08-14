// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PORT_HH
#define XRONOS_SDK_PORT_HH

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/abi/value.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/value_type.hh"
#include "xronos/value/boxing.hh"

namespace xronos::sdk {

namespace detail {

[[nodiscard]] inline auto register_input_port(std::string_view name, const ReactorContext& context) -> std::uint64_t {
  return register_with_location(context, [&]() {
    return get_backend(context).register_input_port(std::string{name}, ContextAccess::get_parent_uid(context));
  });
}

[[nodiscard]] inline auto register_output_port(std::string_view name, const ReactorContext& context) -> std::uint64_t {
  return register_with_location(context, [&]() {
    return get_backend(context).register_output_port(std::string{name}, ContextAccess::get_parent_uid(context));
  });
}

// Hand a serializer to the implementation. Ownership of the callback object
// transfers to the implementation on the call (including on failure), per the
// ABI contract.
inline void set_port_serializer(std::uint64_t port_uid, const ReactorContext& context,
                                abi::PortSerializer* serializer) {
  get_backend(context).set_port_serializer(port_uid, serializer);
}

// Adapts a user serializer to the type-erased values the implementation
// exchanges. It holds one copy of the user's serializer and answers in both
// directions, mirroring the concept: a serializer is one object defining one
// encoding, so it stays one object on the way across.
template <class T, class Serializer> class PortSerializerImpl final : public abi::PortSerializer {
public:
  explicit PortSerializerImpl(Serializer serializer)
      : serializer_{std::move(serializer)} {}

  void serialize(const abi::AnyValue& value, abi::ByteSink& sink) final {
    const auto* payload = xronos::value::get_if<T>(value);
    assert(payload != nullptr && "a serialized port value must hold the port's payload type");
    auto bytes = serializer_.serialize(*payload);
    sink.write(bytes.data(), bytes.size());
  }

  [[nodiscard]] auto deserialize(const std::byte* data, std::size_t size) -> abi::AnyValue final {
    return xronos::value::make<T>(serializer_.deserialize(std::span<const std::byte>{data, size}));
  }

private:
  Serializer serializer_;
};

// Valueless (void) ports carry no payload: serialization writes no bytes,
// and deserialization restores the unit value.
class VoidPortSerializer final : public abi::PortSerializer {
public:
  void serialize([[maybe_unused]] const abi::AnyValue& value, [[maybe_unused]] abi::ByteSink& sink) final {}

  [[nodiscard]] auto deserialize([[maybe_unused]] const std::byte* data, [[maybe_unused]] std::size_t size)
      -> abi::AnyValue final {
    return xronos::value::make<abi::Void>();
  }
};

} // namespace detail

/**
 * @brief Constraints for a serializer that can be used on ports.
 *
 * @p S must convert values of type @p T to bytes and back, and name the byte
 * layout it produces in a static @c encoding member. Conversions are written
 * as calls on a serializer object. This allows conversions to be implemented
 * either as methods or static member functions, and allows a serializer to
 * carry configuration.
 *
 * The encoding name is part of what an exported port declares, and two
 * exported ports connect only when their encoding names are equal. It must therefore identify
 * the layout precisely enough that any two serializers sharing a name are
 * interchangeable, and must be a non-empty compile-time constant.
 *
 * A port names its serializer as a class template, so the constraint applies
 * to @c Serializer<T>.
 *
 * @tparam S The serializer type.
 * @tparam T The value type it converts.
 */
template <class S, class T>
concept IsSerializer = requires(S serializer, const T& value, std::span<const std::byte> data) {
  { serializer.serialize(value) } -> std::same_as<std::vector<std::byte>>;
  { serializer.deserialize(data) } -> std::same_as<T>;
  { S::encoding } -> std::convertible_to<std::string_view>;
};

/**
 * Tag type used as a port's serializer parameter to indicate no serialization.
 *
 * Deliberately not a serializer: it converts nothing and names no encoding, so
 * a port left with it cannot be exported until an encoding is supplied.
 */
template <class T> struct NoSerializer {};

namespace detail {

// The adapter is built owned and released into the call, which takes ownership
// of it: constructing it can throw, releasing cannot, so nothing is leaked on
// the way in.
template <class T, template <class> class Serializer>
void set_port_serializer(std::uint64_t port_uid, const ReactorContext& context) {
  if constexpr (std::is_same_v<void, T>) {
    set_port_serializer(port_uid, context, std::make_unique<VoidPortSerializer>().release());
  } else if constexpr (!std::is_same_v<NoSerializer<T>, Serializer<T>>) {
    set_port_serializer(port_uid, context,
                        std::make_unique<PortSerializerImpl<T, Serializer<T>>>(Serializer<T>{}).release());
  }
}

// Exports a port as part of its node's interface, encoding its values with
// the given serializer. The adapter is constructed before the call, leaving
// only the non-throwing `release`: the implementation takes ownership, so a
// constructed adapter cannot be dropped on the way in.
template <class T, class Serializer>
  requires HasValueTypeName<T> && IsSerializer<Serializer, T>
void export_port(std::uint64_t node_uid, std::uint64_t port_uid, const ReactorContext& context, Serializer serializer) {
  // An empty name identifies nothing, so two ports carrying unrelated types or
  // encodings would compare as a match. Both names are compile-time constants,
  // so the mistake is caught where it is made rather than when the node runs.
  static_assert(!std::string_view{ValueType<T>::name}.empty(),
                "A ValueType specialization must give the type a non-empty name: it is what identifies the type "
                "to other nodes.");
  static_assert(!std::string_view{Serializer::encoding}.empty(),
                "A serializer's encoding must be non-empty: it is what identifies the byte layout to other nodes.");

  auto value_type = std::string{ValueType<T>::name};
  auto encoding = std::string{Serializer::encoding};
  auto owned_serializer = std::make_unique<PortSerializerImpl<T, Serializer>>(std::move(serializer));
  get_backend(context).export_port(node_uid, port_uid, value_type, encoding, owned_serializer.release());
}

// The encoding name a valueless port is exported under. It carries no
// payload, so there is one possible encoding; naming it keeps every exported
// port's identity a pair of names.
inline constexpr std::string_view void_encoding{"void"};

// Valueless counterpart of export_port: no serializer, because there is no
// value to convert.
inline void export_void_port(std::uint64_t node_uid, std::uint64_t port_uid, const ReactorContext& context) {
  auto value_type = std::string{ValueType<void>::name};
  auto encoding = std::string{void_encoding};
  auto owned_serializer = std::make_unique<VoidPortSerializer>();
  get_backend(context).export_port(node_uid, port_uid, value_type, encoding, owned_serializer.release());
}

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
template <class T, template <class> class Serializer = NoSerializer>
  requires std::is_same_v<T, void> || std::is_same_v<Serializer<T>, NoSerializer<T>> ||
           (IsSerializer<Serializer<T>, T> && std::default_initializable<Serializer<T>>)
class InputPort final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  InputPort(std::string_view name, const ReactorContext& context)
      : Element{detail::register_input_port(name, context), name, context} {
    detail::set_port_serializer<T, Serializer>(uid(), context);
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
  requires std::is_same_v<T, void> || std::is_same_v<Serializer<T>, NoSerializer<T>> ||
           (IsSerializer<Serializer<T>, T> && std::default_initializable<Serializer<T>>)
class OutputPort final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the port.
   * @param context The containing reactor's context.
   */
  OutputPort(std::string_view name, const ReactorContext& context)
      : Element{detail::register_output_port(name, context), name, context} {
    detail::set_port_serializer<T, Serializer>(uid(), context);
  }
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PORT_HH
