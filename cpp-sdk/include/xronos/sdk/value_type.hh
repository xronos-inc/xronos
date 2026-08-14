// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_VALUE_TYPE_HH
#define XRONOS_SDK_VALUE_TYPE_HH

#include <concepts>
#include <cstdint>
#include <string>
#include <string_view>

namespace xronos::sdk {

/**
 * @brief The name identifying @p T across separately built nodes.
 *
 * A C++ type does not survive compilation in a form two independently built
 * nodes can compare. An exported port carries this name as a type identifier
 * instead. Two exported ports are assumed to carry the same value type if and
 * only if their names are equal.
 *
 * The SDK names the built-in value types itself:
 *
 * | Type | Name | Type | Name |
 * | --- | --- | --- | --- |
 * | `void` | `void` | `std::uint8_t` | `u8` |
 * | `bool` | `bool` | `std::uint16_t` | `u16` |
 * | `char` | `char` | `std::uint32_t` | `u32` |
 * | `std::int8_t` | `i8` | `std::uint64_t` | `u64` |
 * | `std::int16_t` | `i16` | `float` | `f32` |
 * | `std::int32_t` | `i32` | `double` | `f64` |
 * | `std::int64_t` | `i64` | `std::string` | `string` |
 *
 * The integer types use fixed-width spellings so that a name does not vary
 * with the platform's `int` and `long`.
 *
 * Specialize it for your own types in the header that declares them, so every
 * node including that header agrees on the name by construction:
 *
 * @code
 * namespace acme { struct Pose { double x; double y; }; }
 *
 * template <> struct xronos::sdk::ValueType<acme::Pose> {
 *   static constexpr std::string_view name = "acme.Pose";
 * };
 * @endcode
 *
 * Pick a name unlikely to collide with an unrelated project's type — a dotted
 * path through your organization and package, as above, works well. It must be
 * a non-empty compile-time constant.
 *
 * @tparam T The value type being named.
 */
template <class T> struct ValueType;

/**
 * @brief Satisfied when @p T has a ValueType specialization naming it.
 *
 * Exporting a port requires this, so a missing specialization is reported
 * against the type rather than deep in the export machinery.
 */
template <class T>
concept HasValueTypeName = requires {
  { ValueType<T>::name } -> std::convertible_to<std::string_view>;
};

// The built-in names. Integer types use fixed-width spellings so the identity
// does not vary with the platform's `int` and `long`; the character and `bool`
// types are named in their own right, as distinct value types even where their
// widths coincide.
//
// Hidden from the generated documentation: a page per specialization would say
// only what the primary template already lists, so the names are given there.
/// @cond
template <> struct ValueType<void> {
  static constexpr std::string_view name = "void";
};
template <> struct ValueType<bool> {
  static constexpr std::string_view name = "bool";
};
template <> struct ValueType<char> {
  static constexpr std::string_view name = "char";
};
template <> struct ValueType<std::int8_t> {
  static constexpr std::string_view name = "i8";
};
template <> struct ValueType<std::int16_t> {
  static constexpr std::string_view name = "i16";
};
template <> struct ValueType<std::int32_t> {
  static constexpr std::string_view name = "i32";
};
template <> struct ValueType<std::int64_t> {
  static constexpr std::string_view name = "i64";
};
template <> struct ValueType<std::uint8_t> {
  static constexpr std::string_view name = "u8";
};
template <> struct ValueType<std::uint16_t> {
  static constexpr std::string_view name = "u16";
};
template <> struct ValueType<std::uint32_t> {
  static constexpr std::string_view name = "u32";
};
template <> struct ValueType<std::uint64_t> {
  static constexpr std::string_view name = "u64";
};
template <> struct ValueType<float> {
  static constexpr std::string_view name = "f32";
};
template <> struct ValueType<double> {
  static constexpr std::string_view name = "f64";
};
template <> struct ValueType<std::string> {
  static constexpr std::string_view name = "string";
};
/// @endcond

} // namespace xronos::sdk

#endif // XRONOS_SDK_VALUE_TYPE_HH
