// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_ELEMENT_HH
#define XRONOS_SDK_ELEMENT_HH

#include <concepts>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "xronos/abi/backend.hh"
#include "xronos/abi/exceptions.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/detail/source_location.hh"

namespace xronos::sdk {

/**
 * Possible value types for an attribute.
 */
using AttributeValue = std::variant<std::string, bool, std::int64_t, double>;

/**
 * Exception that is thrown when creating an element with an invalid name.
 *
 * A name is invalid if it is empty, if it contains whitespace or any of
 * `. , / * $ ? # @`, or if it is already in use within the same parent.
 */
using InvalidNameError = abi::InvalidNameError;

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
  [[nodiscard]] auto name() const noexcept -> const std::string& { return name_; }

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
  [[nodiscard]] auto fqn() const noexcept -> const std::string& { return fqn_; }

  /**
   * Get the element's unique ID.
   *
   * Returns an integer ID that is unique within an environment.
   */
  [[nodiscard]] auto uid() const noexcept -> std::uint64_t { return uid_; }

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
  auto add_attribute(std::string_view key, const AttributeValue& value) noexcept -> bool {
    auto& backend = program_context_->backend();
    const std::string key_string{key};
    return std::visit(
        [&](const auto& alternative) -> bool { return backend.add_attribute(uid_, key_string, alternative); }, value);
  }

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

  /** Virtual destructor. */
  virtual ~Element() = default;

  /** Move constructor. */
  Element(Element&&) = default;
  /** @internal */
  Element(const Element&) = delete;
  /** Move assignment operator. */
  auto operator=(Element&&) -> Element& = default;
  /** @internal */
  auto operator=(const Element&) -> Element& = delete;

protected:
  /** @internal */
  Element(std::uint64_t uid, std::string_view name, const Context& context)
      : uid_{uid}
      , name_{name}
      , program_context_{detail::ContextAccess::get_program_context(context)} {
    auto& backend = program_context_->backend();
    fqn_ = backend.element_fqn(uid);
    backend.register_source_location(uid, detail::ContextAccess::get_source_location(context).to_abi());
  }

  /** @internal */
  [[nodiscard]] auto program_context() const noexcept -> const auto& { return program_context_; }

private:
  std::uint64_t uid_;
  std::string name_;
  std::string fqn_;
  std::shared_ptr<detail::ProgramContext> program_context_;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_ELEMENT_HH
