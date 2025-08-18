// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_ELEMENT_HH
#define XRONOS_SDK_ELEMENT_HH

#include <concepts>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

namespace detail {

struct RuntimeElementDeleter {
  void operator()(runtime::ReactorElement* element);
};

// Using a custom deleter allows us to move the pointer without requiring a
// complete definition of runtime::ReactorElement. This is necessary for hiding
// the runtime code from the templated SDK classes (pImpl).
using RuntimeElementPtr = std::unique_ptr<runtime::ReactorElement, RuntimeElementDeleter>;

template <class RuntimeElement>
  requires(std::is_base_of_v<runtime::ReactorElement, RuntimeElement>)
[[nodiscard]] auto get_runtime_instance(const Element& element) -> RuntimeElement&;

template <class RuntimeElement, class... Args>
  requires(std::is_base_of_v<runtime::ReactorElement, RuntimeElement>)
auto make_runtime_element_pointer(Args&&... args) {
  return RuntimeElementPtr(new RuntimeElement(std::forward<Args>(args)...));
}

} // namespace detail

/**
 * Possible value types for an attribute.
 */
using AttributeValue = std::variant<std::string, bool, std::int64_t, double>;

/**
 * Base class for all reactor elements.
 *
 * Reactor elements are objects that can be contained by reactors and that have
 * special meaning to the Xronos SDK.
 */
class Element {
public:
  /**
   * Get the element's name.
   *
   * @returns The element's name.
   */
  [[nodiscard]] auto name() const noexcept -> const std::string&;

  /**
   * Get the element's fully qualified name.
   *
   * The fully qualified name (FQN) represents the containment hierarchy. It
   * consists of the containing reactor's FQN plus the element's name separated
   * by a `.`. For top-level reactors (those owned by the Environment), the FQN
   * is equal to the name.
   *
   * @returns The element's fully qualified name.
   */
  [[nodiscard]] auto fqn() const noexcept -> const std::string&;

  /**
   * Annotate an element with an attribute.
   *
   * Adding the attribute only succeeds, if the given key has not been set
   * before on the same element.
   *
   * See <a href="../../telemetry.html#attributes">Attributes</a> for more information.
   *
   * @param key The name of the attribute to add.
   * @param value The value of the attribute to add.
   * @returns `true` if the attribute was successfully added.
   * @see add_attributes()
   */
  auto add_attribute(std::string_view key, const AttributeValue& value) noexcept -> bool;

  /**
   * Annotate an element with multiple attributes.
   *
   * Adding the attributes only succeeds, if the given key has not been set
   * before on the same element.
   *
   * See <a href="../../telemetry.html#attributes">Attributes</a> for more information.
   *
   * @tparam R Type of the range.
   * @param range Range of key-value-pairs to be added as attributes.
   * @returns `true` if all attributes were successfully added.
   * @see add_attribute()
   */
  template <std::ranges::input_range R>
    requires requires(std::ranges::range_value_t<R> pair) {
      { pair.first } -> std::convertible_to<std::string_view>;
      { pair.second } -> std::convertible_to<AttributeValue>;
    }
  auto add_attributes(const R& range) noexcept -> bool {
    bool success{true};
    for (const auto& [key, value] : range) {
      success |= add_attribute(key, value);
    }
    return success;
  }

  /**
   * @overload
   *
   * @details Accepts an initializer list of key value pairs.
   */
  auto add_attributes(std::initializer_list<std::pair<std::string_view, AttributeValue>> attributes) -> bool {
    return add_attributes(
        std::span<const std::pair<std::string_view, AttributeValue>>(attributes.begin(), attributes.size()));
  }

  /**
   * Virtual destructor.
   */
  virtual ~Element() = default;

protected:
  /** @internal */
  Element(detail::RuntimeElementPtr runtime_instance, Context context);

private:
  detail::RuntimeElementPtr runtime_instance_;
  std::reference_wrapper<telemetry::AttributeManager> attribute_manager_;

  template <class RuntimeElement>
    requires(std::is_base_of_v<runtime::ReactorElement, RuntimeElement>)
  friend auto detail::get_runtime_instance(const Element& element) -> RuntimeElement&;
};

} // namespace xronos::sdk

namespace xronos::sdk::detail {

template <class RuntimeElement>
  requires(std::is_base_of_v<runtime::ReactorElement, RuntimeElement>)
[[nodiscard]] auto get_runtime_instance(const Element& element) -> RuntimeElement& {
  return dynamic_cast<RuntimeElement&>(*element.runtime_instance_);
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_ELEMENT_HH
