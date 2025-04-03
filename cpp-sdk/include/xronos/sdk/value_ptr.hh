// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Smart pointers used by the runtime to protect memory safety and
 * determinism.
 */

#ifndef XRONOS_SDK_VALUE_PTR_HH
#define XRONOS_SDK_VALUE_PTR_HH

#include "xronos/runtime/value_ptr.hh"

namespace xronos::sdk {

/**
 * @brief Smart pointer to a mutable value.
 *
 * @details Manages the lifetime of a value in conjunction with
 * `ImmutableValuePtr`. Implements ownership semantics and enforces unique
 * ownership of a mutable value. `MutableValuePtr` internally wraps around
 * `unique_ptr`. The unique ownership ensures that no other reactor can
 * reference the value while it is allowed to change. In order to share the
 * associated value, an instance of `MutableValuePtr` needs to be converted to
 * an `ImmutableValuePtr`, making the associated value immutable.
 * @tparam T Type of the value managed by this class.
 */
template <class T> using MutableValuePtr = runtime::MutableValuePtr<T>;
/**
 * @brief Smart pointer to an immutable value.
 *
 * @details Manages the lifetime of a value in conjunction with
 * `MutableValuePtr`. Implements ownership semantics and allows shared ownership
 * of an immutable value. `ImmutableValuePtr` internally wraps around
 * `shared_ptr`. The shared ownership semantics enables multiple reactors to
 * share a value, which is only safe if the value is immutable. In order to
 * modify the associated value, an instance of `ImmutableValuePtr` needs to be
 * converted to an `MutableValuePtr`, making the associated value mutable. This
 * can be achieved by calling `get_mutable_copy()`.
 * @tparam T Type of the value managed by this class.
 */
template <class T> using ImmutableValuePtr = runtime::ImmutableValuePtr<T>;

/**
 * @brief Create an instance of `MutableValuePtr`.

 * @details Creates and initializes a new instance of `T` and returns a new
 * `MutableValuePtr` owning this value. This is analogous to
 * `std::make_unique`.
 * @tparam T type of the value to be created
 * @tparam Args types of `T`'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * given `args`.
 * @param args Arguments to be forwarded to `T`'s constructor
 * @return A new mutable value pointer.
 */
template <class T, class... Args> auto make_mutable_value(Args&&... args) -> MutableValuePtr<T> {
  return runtime::make_mutable_value<T>(std::forward<Args>(args)...);
}
/**
 * @brief Create an instance of `ImmutableValuePtr`.
 *
 * @details Creates and initializes a new instance of `T` and returns a new
 * `ImmutableValuePtr` owning this value. This is analogous to
 * `std::make_shared`.
 * @tparam T type of the value to be created
 * @tparam Args types of `T`'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * given `args`.
 * @param args Arguments to be forwarded to `T`'s constructor
 * @return A new immutable value pointer.
 */
template <class T, class... Args> auto make_immutable_value(Args&&... args) -> ImmutableValuePtr<T> {
  return runtime::make_immutable_value<T>(std::forward<Args>(args)...);
}

} // namespace xronos::sdk

#endif // XRONOS_SDK_VALUE_PTR_HH
