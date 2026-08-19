// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_VALUE_HH
#define XRONOS_SDK_VALUE_HH

#include <cassert>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#include "xronos/abi/value.hh"
#include "xronos/value/boxing.hh"

namespace xronos::sdk {

template <class T> class Value;
template <class T> class ValueView;

namespace detail {

// Internal bridge between the typed user-facing value types and the
// type-erased values exchanged with the runtime.
struct ValueAccess {
  template <class T> [[nodiscard]] static auto view(const abi::AnyValue& value) noexcept -> ValueView<T> {
    return ValueView<T>{value};
  }
  template <class T> [[nodiscard]] static auto unwrap(const Value<T>& value) noexcept -> const abi::AnyValue& {
    return value.value_;
  }
  template <class T> [[nodiscard]] static auto unwrap(const ValueView<T>& view) noexcept -> const abi::AnyValue* {
    return view.value_;
  }
};

} // namespace detail

/**
 * An owned, shareable message value.
 *
 * The held value remains valid for as long as the `Value` object itself (or
 * any copy of it) is alive. Copying a
 * `Value` is cheap: values are immutable while shared, so copies share the
 * underlying storage instead of duplicating it.
 *
 * Use a `Value` to keep a value received in a reaction beyond the reaction's
 * execution (by copying the ValueView returned from
 * @ref xronos::sdk::BaseReaction::Trigger "Trigger"`::get()` into a `Value`),
 * to share a value between reactors without copying it, or to send the same
 * value to multiple @ref effects without boxing it again.
 *
 * A default-constructed `Value` is empty and holds nothing.
 *
 * @tparam T The type of the held value.
 */
template <class T> class Value {
  static_assert(std::is_same_v<T, std::remove_cvref_t<T>>, "Value requires a plain value type");
  static_assert(!std::is_same_v<T, void>, "Value<void> is not meaningful; valueless messages carry no value");

public:
  /**
   * Constructs an empty `Value` that holds nothing.
   */
  constexpr Value() noexcept = default;

  /**
   * Constructs a `Value` holding a copy of @p value.
   */
  explicit Value(const T& value)
      : value_{xronos::value::make<T>(value)} {}

  /**
   * Constructs a `Value` holding a value move-constructed from @p value.
   */
  explicit Value(T&& value)
      : value_{xronos::value::make<T>(std::move(value))} {}

  /**
   * Constructs a `Value` from a ValueView, retaining the viewed value.
   *
   * This is the way to keep a value received in a reaction beyond the
   * reaction's execution. If `view` is absent, the constructed `Value` is
   * empty.
   */
  explicit Value(const ValueView<T>& view);

  /**
   * Constructs a `Value` holding a value constructed in place from @p args.
   *
   * This is analogous to `std::make_shared()`.
   */
  template <class... Args> [[nodiscard]] static auto make(Args&&... args) -> Value {
    return Value{xronos::value::make<T>(std::forward<Args>(args)...)};
  }

  /**
   * Constructs a `Value` that shares ownership of an externally managed
   * value instead of copying it.
   *
   * Use this to pass values owned by another framework (for example,
   * messages received from a ROS 2 subscription) into the reactor program
   * without copying the payload. The value must not be modified through any
   * other reference for as long as it is shared -- the same contract that
   * sharing a `std::shared_ptr<const T>` implies everywhere else. To hand
   * over exclusive ownership of a value instead, use from_unique_ptr().
   *
   * A null pointer yields an empty `Value`.
   *
   * @note Whether the payload is shared or copied is a storage detail of the
   * value type: small trivially copyable types are stored by value and are
   * copied out of the given pointer instead (see as_shared_ptr()).
   */
  [[nodiscard]] static auto from_shared_ptr(std::shared_ptr<const T> pointer) -> Value {
    return Value{xronos::value::from_shared_ptr<T>(std::move(pointer))};
  }

  /**
   * Constructs a `Value` that takes over exclusive ownership of an
   * externally allocated value, without copying it.
   *
   * Unlike from_shared_ptr(), no other reference to the value can remain,
   * so no immutability promise from the caller is needed. Custom deleters
   * are preserved and invoked when the last `Value` sharing the storage is
   * destroyed (for example, the message deleters used by ROS 2
   * subscriptions taking `std::unique_ptr` messages).
   *
   * A null pointer yields an empty `Value`.
   *
   * @note Small trivially copyable types are stored by value and are copied
   * out of the given pointer instead (see as_shared_ptr()).
   */
  template <class Deleter> [[nodiscard]] static auto from_unique_ptr(std::unique_ptr<T, Deleter> pointer) -> Value {
    return from_shared_ptr(std::shared_ptr<const T>{std::move(pointer)});
  }

  /**
   * Retrieve the held value as a `std::shared_ptr`.
   *
   * The returned pointer keeps the value alive independently of this
   * `Value`, which makes it suitable for handing values to another framework
   * (for example, publishing to ROS 2). Note that pointer identity is not
   * necessarily carried through: for payload types in shared storage the
   * returned pointer shares ownership with this `Value`, while small
   * trivially copyable payloads are stored by value and are copied into a
   * fresh allocation on every call.
   *
   * @returns A shared pointer to the held value, or `nullptr` if this
   * `Value` is empty.
   */
  [[nodiscard]] auto as_shared_ptr() const -> std::shared_ptr<const T> {
    return xronos::value::as_shared_ptr<T>(value_);
  }

  /**
   * Retrieve a pointer to the held value, or `nullptr` if this `Value` is
   * empty.
   *
   * The pointer remains valid for as long as any `Value` sharing the same
   * storage is alive.
   */
  [[nodiscard]] auto get() const noexcept -> const T* { return xronos::value::get_if<T>(value_); }

  /**
   * Check whether this `Value` holds a value.
   */
  explicit operator bool() const noexcept { return value_.has_value(); }

  /**
   * Check whether this `Value` is empty (`value == nullptr` is equivalent to
   * `!value`).
   */
  friend auto operator==(const Value& value, std::nullptr_t) noexcept -> bool { return !value.value_.has_value(); }

  /**
   * Access the held value.
   *
   * The behavior is undefined if this `Value` is empty (asserted in debug
   * builds).
   */
  auto operator*() const noexcept -> const T& {
    const T* payload = get();
    assert(payload != nullptr && "dereferenced an empty Value");
    return *payload;
  }

  /**
   * Access members of the held value.
   *
   * The behavior is undefined if this `Value` is empty (asserted in debug
   * builds).
   */
  auto operator->() const noexcept -> const T* {
    const T* payload = get();
    assert(payload != nullptr && "dereferenced an empty Value");
    return payload;
  }

private:
  explicit Value(abi::AnyValue&& value) noexcept
      : value_{std::move(value)} {}

  abi::AnyValue value_{};

  friend struct detail::ValueAccess;
};

/**
 * A borrowed view of a message value.
 *
 * A `ValueView` does not own the value it refers to: it forwards to a value
 * stored by the runtime, which remains valid for the duration of the current
 * reaction handler. Reading through a view performs no copy.
 *
 * Views cannot be copied or moved: a view is a scope-bound borrow, and
 * retaining one beyond the reaction handler it was obtained in would leave
 * it dangling. Initializing a local from a fresh view
 * (`auto view = input.get();`) works -- that is guaranteed copy elision,
 * not a copy. To keep the viewed value, copy the view into a @ref Value :
 *
 * ```cpp
 * if (auto view = input_.get()) {
 *   self().last_input_ = Value<Message>{view};
 * }
 * ```
 *
 * A default-constructed (or absent) view refers to nothing; this is what
 * @ref xronos::sdk::BaseReaction::Trigger "Trigger"`::get()` returns when no
 * event is present.
 *
 * @tparam T The type of the viewed value.
 */
template <class T> class ValueView {
  static_assert(std::is_same_v<T, std::remove_cvref_t<T>>, "ValueView requires a plain value type");
  static_assert(!std::is_same_v<T, void>, "ValueView<void> is not meaningful; valueless messages carry no value");

public:
  /**
   * Constructs an absent view that refers to nothing.
   */
  constexpr ValueView() noexcept = default;

  // Views are scope-bound borrows; copying or moving one is the first step
  // of retaining it beyond its validity, so neither is allowed. Fresh views
  // returned from get() initialize locals via guaranteed copy elision.
  ValueView(const ValueView&) = delete;
  ValueView(ValueView&&) = delete;
  auto operator=(const ValueView&) -> ValueView& = delete;
  auto operator=(ValueView&&) -> ValueView& = delete;
  ~ValueView() = default;

  /**
   * Retrieve a pointer to the viewed value, or `nullptr` if the view is
   * absent.
   *
   * The pointer is only valid for the duration of the current reaction
   * handler.
   */
  [[nodiscard]] auto get() const noexcept -> const T* { return payload_; }

  /**
   * Check whether the view refers to a value.
   *
   * For a view returned by a trigger, this indicates whether an event with a
   * value is currently present.
   */
  explicit operator bool() const noexcept { return payload_ != nullptr; }

  /**
   * Check whether the view is absent (`view == nullptr` is equivalent to
   * `!view`).
   */
  friend auto operator==(const ValueView& view, std::nullptr_t) noexcept -> bool { return view.payload_ == nullptr; }

  /**
   * Access the viewed value.
   *
   * The behavior is undefined if the view is absent (asserted in debug
   * builds).
   */
  auto operator*() const noexcept -> const T& {
    assert(payload_ != nullptr && "dereferenced an absent ValueView");
    return *payload_;
  }

  /**
   * Access members of the viewed value.
   *
   * The behavior is undefined if the view is absent (asserted in debug
   * builds).
   */
  auto operator->() const noexcept -> const T* {
    assert(payload_ != nullptr && "dereferenced an absent ValueView");
    return payload_;
  }

  /**
   * Retrieve the viewed value as a `std::shared_ptr`.
   *
   * Unlike the view itself, the returned pointer keeps the value alive
   * beyond the current reaction handler, which makes it suitable for handing
   * a received value to another framework (for example, publishing to
   * ROS 2). Note that pointer identity is not necessarily carried through:
   * for payload types in shared storage the returned pointer shares
   * ownership with the stored value, while small trivially copyable payloads
   * are stored by value and are copied into a fresh allocation on every
   * call.
   *
   * @returns A shared pointer to the viewed value, or `nullptr` if the view
   * is absent.
   */
  [[nodiscard]] auto as_shared_ptr() const -> std::shared_ptr<const T> {
    return value_ != nullptr ? xronos::value::as_shared_ptr<T>(*value_) : nullptr;
  }

private:
  explicit ValueView(const abi::AnyValue& value) noexcept
      : value_{&value}
      , payload_{xronos::value::get_if<T>(value)} {}

  const abi::AnyValue* value_{nullptr};
  const T* payload_{nullptr};

  friend class Value<T>;
  friend struct detail::ValueAccess;
};

template <class T>
Value<T>::Value(const ValueView<T>& view)
    : value_{view.value_ != nullptr ? *view.value_ : abi::AnyValue{}} {}

} // namespace xronos::sdk

#endif // XRONOS_SDK_VALUE_HH
