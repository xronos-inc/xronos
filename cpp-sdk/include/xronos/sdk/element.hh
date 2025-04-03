// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `Element` class.
 */

#ifndef XRONOS_SDK_ELEMENT_HH
#define XRONOS_SDK_ELEMENT_HH

#include <memory>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

class Element;

using AttributeValue = std::variant<std::string, bool, std::int64_t, double>;

namespace detail {

template <class RuntimeElement>
  requires(std::is_base_of_v<runtime::ReactorElement, RuntimeElement>)
[[nodiscard]] auto get_runtime_instance(const Element& element) -> RuntimeElement&;

} // namespace detail

/**
 * @brief Base class for all reactor elements.
 *
 * @details Reactor elements are objects that can be contained by reactors and
 * that have special meaning to the Xronos framework.
 */
class Element {
public:
  /**
   * @brief The name of the current element.
   *
   * @details A name should be unique within the current reactor and should
   * contain only word characters (alphanumeric and underscore).
   * @return The current element's name.
   */
  [[nodiscard]] auto name() const noexcept -> const std::string&;
  /**
   * @brief The fully qualified name of the current element.
   *
   * @details This should be globally unique and is computed as the
   * "."-separated concatenation of the all parent reactors, starting from the
   * top-level environment, followed by the current element's name.
   * @return The current element's fully qualified name.
   */
  [[nodiscard]] auto fqn() const noexcept -> const std::string&;

  auto add_attribute(std::string_view key, const AttributeValue& value) noexcept -> bool;

protected:
  Element(std::unique_ptr<runtime::ReactorElement> runtime_instance, Context context);

private:
  std::unique_ptr<runtime::ReactorElement> runtime_instance_;
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
