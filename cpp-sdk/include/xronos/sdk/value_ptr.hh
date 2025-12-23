// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_VALUE_PTR_HH
#define XRONOS_SDK_VALUE_PTR_HH

#include <cstddef>
#include <memory>
#include <type_traits>

namespace xronos::sdk {

namespace detail {

template <class T, bool is_trivial> class ImmutableValuePtr {};
template <class T, bool is_trivial> class MutableValuePtr {};

constexpr std::size_t SIZE_THRESHOLD = 64;

template <class T> constexpr auto is_trivial() -> bool {
  return std::is_default_constructible_v<T> && std::is_trivially_copyable_v<T> && sizeof(T) <= SIZE_THRESHOLD;
}
template <> constexpr auto is_trivial<void>() -> bool { return true; }

} // namespace detail

/**
 * Smart pointer to a mutable value.
 *
 * This behaves similarly to `std::unique_ptr<T>`. While the value can be
 * modified, there may at most one owner.
 */
template <class T> using MutableValuePtr = detail::MutableValuePtr<T, detail::is_trivial<T>()>;

/**
 * Smart pointer to an immutable value.
 *
 * Ownership may be shared, but none of the owners may modify the value. This
 * behaves similarly to `std::shared_ptr<const T>`.
 */
template <class T> using ImmutableValuePtr = detail::ImmutableValuePtr<T, detail::is_trivial<T>()>;

/**
 * Create an instance of ImmutableValuePtr.
 *
 * Creates and initializes a new instance of `T` and returns a new
 * ImmutableValuePtr owning this value. This is analogues to
 * `std::make_shared()`.
 *
 * @tparam T The type of the value to be created.
 * @tparam Args The types of `T`'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * given @p args.
 * @param args Arguments to be forwarded to `T`'s constructor.
 * @returns A new immutable value pointer owning an instance of `T`.
 */
template <class T, class... Args> auto make_immutable_value(Args&&... args) -> ImmutableValuePtr<T> {
  if constexpr (detail::is_trivial<T>()) {
    return ImmutableValuePtr<T>(T(std::forward<Args>(args)...));
  } else {
    return ImmutableValuePtr<T>(std::make_shared<T>(std::forward<Args>(args)...));
  }
}

/**
 * Create an instance of MutableValuePtr.
 *
 * Creates and initializes a new instance of `T` and returns a new
 * MutableValuePtr owning this value. This is analogues to
 * `std::make_unique()`.
 *
 * @tparam T The type of the value to be created.
 * @tparam Args The types of `T`'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * given @p args.
 * @param args Arguments to be forwarded to `T`'s constructor.
 * @returns A new mutable value pointer owning an instance of `T`.
 */
template <class T, class... Args> auto make_mutable_value(Args&&... args) -> MutableValuePtr<T> {
  if constexpr (detail::is_trivial<T>()) {
    return MutableValuePtr<T>(T(std::forward<Args>(args)...));
  } else {
    return MutableValuePtr<T>(std::make_unique<T>(std::forward<Args>(args)...));
  }
}

namespace detail {

/**
 * @brief Smart pointer to a mutable value.
 *
 * Manages the lifetime of a value in conjunction with `ImmutableValuePtr`.
 * Implements ownership semantics and enforces unique ownership of a mutable
 * value. `MutableValuePtr` internally wraps around `std::unique_ptr`. The
 * unique ownership ensures that no other reactor can reference the value while
 * it is allowed to change. In order to share the associated value, an instance
 * of `MutableValuePtr` needs to be converted to an `ImmutableValuePtr` making
 * the associated value immutable.
 *
 * @tparam T Type of the value managed by this class.
 */
template <class T> class MutableValuePtr<T, false> {
private:
  /// The internal unique smart pointer that this class builds upon.
  std::unique_ptr<T> internal_ptr;

  explicit MutableValuePtr(std::unique_ptr<T>&& value)
      : internal_ptr(std::move(value)) {}

public:
  /**
   * Default constructor.
   *
   * Constructs a :class:`MutableValuePtr` that owns nothing.
   */
  constexpr MutableValuePtr() = default;
  ~MutableValuePtr() = default;

  /**
   * Copy constructor (Deleted).
   *
   * Since `MutableValuePtr` enforces unique ownership, there cannot
   * be two instances pointing to the same object and therefore copying cannot
   * be allowed,
   */
  MutableValuePtr(const MutableValuePtr&) = delete;

  /**
   * Move constructor.
   *
   * Constructs a `MutableValuePtr` by transferring ownership from another
   * `MutableValuePtr` instance `ptr`. `ptr` looses ownership and will own
   * nothing.
   */
  MutableValuePtr(MutableValuePtr&& ptr) noexcept = default;

  /**
   * Constructor from `nullptr`.
   *
   * Constructs a `MutableValuePtr<T>` that owns nothing.
   */
  explicit constexpr MutableValuePtr(std::nullptr_t)
      : internal_ptr(nullptr) {}

  /**
   * Move assignment operator.
   *
   * Transfers ownership from `ptr` to this `MutableValuePtr` instance. If this
   * instance previously owned a value, the value is deleted.
   *
   */
  auto operator=(MutableValuePtr&& ptr) noexcept -> MutableValuePtr& {
    this->internal_ptr = std::move(ptr.internal_ptr);
    return *this;
  }

  /**
   * Copy assignment operator (Deleted).
   *
   * Since `MutableValuePtr` enforces unique ownership, there cannot be two
   * instances pointing to the same object and therefore copying cannot be
   * allowed,
   */
  auto operator=(const MutableValuePtr& ptr) -> MutableValuePtr& = delete;

  /**
   * Assignment operator from `nullptr`.
   *
   * Releases ownership. If this instance previously owned a value, the value is
   * deleted.
   */
  auto operator=(std::nullptr_t) noexcept -> MutableValuePtr& {
    this->internal_ptr = nullptr;
    return *this;
  }

  /**
   * Retrieve a raw pointer to the managed value.
   */
  [[nodiscard]] auto get() const noexcept -> T* { return internal_ptr.get(); }

  /**
   * Cast to `bool`. Checks if there is an associated value.
   *
   * @return `false` if there is no associated value (`get() == nullptr`),
   * `true` otherwise
   */
  explicit operator bool() const { return get() == nullptr; }

  /**
   * Dereference the pointer to the managed value.
   *
   * The behavior is undefined if `get() == nullptr`.
   */
  auto operator*() const -> T& { return *get(); }
  /**
   * Dereference the pointer to the managed value.
   *
   * Provides access to members of the associated value via `->`. The
   * behavior is undefined if `get() == nullptr`.
   */
  auto operator->() const -> T* { return get(); }

  // Give ImmutableValuePtr access to the private constructor. This is required
  // for creating a MutableValuePtr from an ImmutableValuePtr in
  // get_mutable_copy()
  friend class ImmutableValuePtr<T, false>;

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend auto sdk::make_mutable_value(Args&&... args) -> sdk::MutableValuePtr<U>;
};

/**
 * @brief Smart pointer to an immutable value.
 *
 * Manages the lifetime of a value in conjunction with
 * `MutableValuePtr`. Implements ownership semantics and allows shared
 * ownership of an immutable value. `ImmutableValuePtr` internally
 * wraps around `std::shared_ptr`. The shared ownership semantics
 * enables multiple reactors to share a value which is only safe if the value
 * is immutable.  In order to modify the associated value, an instance of
 * `ImmutableValuePtr` needs to be converted to an
 * `MutableValuePtr` making the associated value mutable. This can be
 * achieved by calling `get_mutable_copy()`.
 *
 * @tparam T type of the value managed by this class.
 */
template <class T> class ImmutableValuePtr<T, false> {
public:
  /// A type alias that adds ``const`` to ``T``
  using const_T = std::add_const_t<T>;

private:
  /// The internal shared smart pointer that this class builds upon.
  std::shared_ptr<T> internal_ptr;

  explicit ImmutableValuePtr(std::shared_ptr<T>&& value)
      : internal_ptr(std::move(value)) {}

public:
  /**
   * Default constructor.
   *
   * Constructs an `ImmutableValuePtr<T>` that owns nothing.
   */
  constexpr ImmutableValuePtr()
      : internal_ptr(nullptr) {}

  ~ImmutableValuePtr() = default;

  /**
   * Copy constructor.
   *
   * Constructs an `ImmutableValuePtr` by copying another `ImmutableValuePtr`
   * instance `ptr`. Both pointers have the same associated value and,
   * therefore, share ownership.
   */
  ImmutableValuePtr(const ImmutableValuePtr& ptr) = default;
  /**
   * Move constructor.
   *
   * Constructs an `ImmutableValuePtr` by transferring ownership from
   * another `ImmutableValuePtr` instance `ptr`. `ptr` looses
   * ownership and will own nothing.
   */
  ImmutableValuePtr(ImmutableValuePtr&& ptr) noexcept = default;
  /**
   * Constructor from `nullptr`.
   *
   * Constructs an `ImmutableValuePtr<T>` that owns nothing.
   */
  explicit constexpr ImmutableValuePtr(std::nullptr_t)
      : internal_ptr(nullptr) {}
  /**
   * Move constructor from `MutableValuePtr`.
   *
   * Constructs an `ImmutableValuePtr` by transferring ownership from a
   * `MutableValuePtr` instance `ptr`. `ptr` looses ownership and
   * will own nothing. This effectively converts the mutable value initially
   * associated with `ptr` to an immutable value.
   */
  explicit ImmutableValuePtr(MutableValuePtr<T, false>&& ptr)
      : internal_ptr(std::move(std::move(ptr).internal_ptr)) {}

  /**
   * Assignment operator from `nullptr`.
   *
   * Releases ownership. If this instance previously owned a value that is not
   * owned by any other instance of `ImmutableValuePtr`, the value is deleted.
   */
  auto operator=(std::nullptr_t) -> ImmutableValuePtr& {
    this->internal_ptr = nullptr;
    return *this;
  }
  /**
   * Assignment operator from another `ImmutableValuePtr`.
   *
   * Replaces the managed value of this instance by the one managed by
   * `ptr`. Both instances share the ownership. If this instance previously
   * owned a value that is not owned by any other instance of
   * `ImmutableValuePtr`, the value is deleted.
   */
  auto operator=(const ImmutableValuePtr& ptr) -> ImmutableValuePtr& = default;

  /**
   * Move assignment operator from another `ImmutableValuePtr`.
   *
   * Replaces the managed value of this instance by the one managed by `ptr`.
   * This moves the ownership from `ptr` to this instance. If this instance
   * previously owned a value that is not owned by any other instance of
   * `ImmutableValuePtr`, the value is deleted.
   */
  auto operator=(ImmutableValuePtr&& ptr) noexcept -> ImmutableValuePtr& = default;

  /**
   * Retrieve a raw pointer to the managed value.
   *
   * Since the associated value is immutable, this only provides const access
   * to the value.
   */
  [[nodiscard]] auto get() const -> const_T* { return internal_ptr.get(); }

  /**
   * Cast to `bool`. Checks if there is an associated value.
   *
   * @return `false` if there is no associated value (`get() == nullptr`),
   * `true` otherwise
   */
  explicit operator bool() const { return get() == nullptr; }

  /**
   * Dereference the pointer to the managed value.
   *
   * Since the associated value is immutable, this only provides const access
   * to the value.
   *
   * The behavior is undefined if `get() == nullptr`.
   */
  auto operator*() const -> const_T& { return *get(); }
  /**
   * Dereference the pointer to the managed value.
   *
   * Since the associated value is immutable, this only provides const access
   * to the value.
   *
   * Provides access to members of the associated value via `->`. The
   * behavior is undefined if `get() == nullptr`.
   */
  auto operator->() const -> const_T* { return get(); }

  /**
   * Create a mutable copy of the value associated with this instance.
   *
   * This is the only allowed mechanism to convert a `ImmutableValuePtr` to a
   * `MutableValuePtr`. In fact, it does not perform a conversion but copies the
   * associated value of this instance and gives ownership of the copy to a
   * newly created :class:`MutableValuePtr`.
   *
   * Requires that `T` is copy constructable. The behavior is undefined if
   * `get() == nullptr`.
   * @return a mutable value pointer
   */
  [[nodiscard]] auto get_mutable_copy() const -> MutableValuePtr<T, false> {
    return MutableValuePtr<T, false>(std::make_unique<T>(*internal_ptr));
  }

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend auto sdk::make_immutable_value(Args&&... args) -> sdk::ImmutableValuePtr<U>;
};

template <class T> class MutableValuePtr<T, true> {
private:
  T value_{};
  bool valid_{false};

  explicit MutableValuePtr(const T& value)
      : value_{value}
      , valid_{true} {}

public:
  constexpr MutableValuePtr() = default;
  ~MutableValuePtr() = default;
  MutableValuePtr(const MutableValuePtr&) = delete;
  MutableValuePtr(MutableValuePtr&& ptr) noexcept = default;

  explicit constexpr MutableValuePtr(std::nullptr_t) {}

  auto operator=(MutableValuePtr&& ptr) noexcept -> MutableValuePtr& = default;
  auto operator=(const MutableValuePtr& ptr) -> MutableValuePtr& = delete;

  auto operator=(std::nullptr_t) noexcept -> MutableValuePtr& {
    valid_ = false;
    return *this;
  }

  [[nodiscard]] auto get() noexcept -> T* { return valid_ ? &value_ : nullptr; }
  [[nodiscard]] auto get() const noexcept -> const T* { return valid_ ? &value_ : nullptr; }

  explicit operator bool() const { return valid_; }

  auto operator*() -> T& { return value_; }
  auto operator*() const -> const T& { return value_; }

  auto operator->() -> T* { return get(); }
  auto operator->() const -> const T* { return get(); }

  // Give ImmutableValuePtr access to the private constructor. This is required
  // for creating a MutableValuePtr from an ImmutableValuePtr in
  // get_mutable_copy()
  friend class ImmutableValuePtr<T, true>;

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args> friend auto sdk::make_mutable_value(Args&&... args) -> sdk::MutableValuePtr<U>;
};

template <class T> class ImmutableValuePtr<T, true> {
public:
  /// A type alias that adds ``const`` to ``T``
  using const_T = typename std::add_const_t<T>;

private:
  T value_{};
  bool valid_{false};

  explicit ImmutableValuePtr(T value)
      : value_{value}
      , valid_{true} {}

public:
  constexpr ImmutableValuePtr() = default;
  ~ImmutableValuePtr() = default;
  ImmutableValuePtr(const ImmutableValuePtr& ptr) = default;
  ImmutableValuePtr(ImmutableValuePtr&& ptr) noexcept = default;

  explicit constexpr ImmutableValuePtr(std::nullptr_t) {}
  // NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
  explicit ImmutableValuePtr(MutableValuePtr<T, true>&& ptr)
      : value_(std::move(ptr.value_))
      , valid_(ptr.valid_) {
    ptr.valid_ = false;
  }

  auto operator=(std::nullptr_t) -> ImmutableValuePtr& {
    this->valid_ = false;
    return *this;
  }
  auto operator=(const ImmutableValuePtr& ptr) -> ImmutableValuePtr& = default;
  auto operator=(ImmutableValuePtr&& ptr) noexcept -> ImmutableValuePtr& = default;

  [[nodiscard]] auto get() const -> const_T* { return valid_ ? &value_ : nullptr; }

  explicit operator bool() const { return valid_; }

  auto operator*() const -> const_T& { return value_; }
  auto operator->() const -> const_T* { return get(); }

  [[nodiscard]] auto get_mutable_copy() const -> MutableValuePtr<T, true> { return MutableValuePtr<T, true>(*get()); }

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args> friend auto sdk::make_immutable_value(Args&&... args) -> sdk::ImmutableValuePtr<U>;
};

// Comparison operators

template <class T, class U, bool is_trivial>
auto operator==(const MutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator==(const ImmutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator==(const ImmutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator==(const MutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, bool is_trivial>
auto operator==(const MutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) noexcept -> bool {
  return ptr1.get() == nullptr;
}
template <class T, bool is_trivial>
auto operator==(std::nullptr_t, const MutableValuePtr<T, is_trivial>& ptr2) noexcept -> bool {
  return ptr2.get() == nullptr;
}
template <class T, bool is_trivial>
auto operator==(const ImmutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) noexcept -> bool {
  return ptr1.get() == nullptr;
}
template <class T, bool is_trivial>
auto operator==(std::nullptr_t, const ImmutableValuePtr<T, is_trivial>& ptr1) noexcept -> bool {
  return ptr1.get() == nullptr;
}

template <class T, class U, bool is_trivial>
auto operator!=(const MutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() != ptr2.get();
}

template <class T, class U, bool is_trivial>
auto operator!=(const ImmutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) -> bool {
  return ptr1.get() != ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator!=(const ImmutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) -> bool {
  return ptr1.get() != ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator!=(const MutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) -> bool {
  return ptr1.get() != ptr2.get();
}
template <class T, bool is_trivial>
auto operator!=(const MutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) -> bool {
  return ptr1.get() != nullptr;
}
template <class T, bool is_trivial>
auto operator!=(std::nullptr_t, const MutableValuePtr<T, is_trivial>& ptr1) -> bool {
  return ptr1.get() != nullptr;
}
template <class T, bool is_trivial>
auto operator!=(const ImmutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) -> bool {
  return ptr1.get() != nullptr;
}
template <class T, bool is_trivial>
auto operator!=(std::nullptr_t, const ImmutableValuePtr<T, is_trivial>& ptr1) -> bool {
  return ptr1.get() != nullptr;
}

} // namespace detail

} // namespace xronos::sdk

#endif
