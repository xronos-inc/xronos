// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of context objects that can be used for constructing
 * reactors.
 */

#ifndef XRONOS_SDK_CONTEXT_HH
#define XRONOS_SDK_CONTEXT_HH

#include <source_location>
#include <string_view>
#include <variant>

#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

class EnvironmentContext;
class ReactorContext;

using Context = std::variant<EnvironmentContext, ReactorContext>;

namespace detail {

[[nodiscard]] constexpr auto get_reactor_instance(ReactorContext context) -> runtime::Reactor&;
[[nodiscard]] constexpr auto get_environment(EnvironmentContext context) -> Environment&;
[[nodiscard]] constexpr auto get_environment(ReactorContext context) -> Environment&;
[[nodiscard]] constexpr auto get_environment(Context context) -> Environment&;

void store_source_location(EnvironmentContext context, std::uint64_t uid, std::string_view fqn);
void store_source_location(ReactorContext context, std::uint64_t uid, std::string_view fqn);
void store_source_location(Context context, std::uint64_t uid, std::string_view fqn);

} // namespace detail

/**
 * @brief Opaque object used by top-level reactors at construction time.
 *
 * @details This object usually should not be accessed directly. Instead, this
 * object should be used as an argument of reactor constructors.
 */
class EnvironmentContext {
private:
  constexpr EnvironmentContext(Environment& environment, std::source_location source_location)
      : environment_(environment)
      , source_location_{source_location} {}
  std::reference_wrapper<Environment> environment_;
  std::source_location source_location_;

  friend Environment;
  friend constexpr auto detail::get_environment(EnvironmentContext context) -> Environment&;
  friend void detail::store_source_location(EnvironmentContext context, std::uint64_t uid, std::string_view fqn);
};

/**
 * @brief Opaque object used by reactors at construction time.
 *
 * @details This object usually should not be accessed directly. Instead, this
 * object should be used as an argument of reactor constructors.
 */
class ReactorContext {
private:
  constexpr ReactorContext(runtime::Reactor& reactor_instance, Environment& environment,
                           std::source_location source_location)
      : reactor_instance_{reactor_instance}
      , environment_{environment}
      , source_location_{source_location} {}

  std::reference_wrapper<runtime::Reactor> reactor_instance_;
  std::reference_wrapper<Environment> environment_;
  std::source_location source_location_;

  friend Reactor;
  friend constexpr auto detail::get_reactor_instance(ReactorContext context) -> runtime::Reactor&;
  friend constexpr auto detail::get_environment(ReactorContext context) -> Environment&;
  friend void detail::store_source_location(ReactorContext context, std::uint64_t uid, std::string_view fqn);
};

namespace detail {

[[nodiscard]] constexpr auto get_reactor_instance(ReactorContext context) -> runtime::Reactor& {
  return context.reactor_instance_;
}

[[nodiscard]] constexpr auto get_environment(EnvironmentContext context) -> Environment& {
  return context.environment_;
}

[[nodiscard]] constexpr auto get_environment(ReactorContext context) -> Environment& { return context.environment_; }

[[nodiscard]] constexpr auto get_environment(Context context) -> Environment& {
  return std::visit([](auto& context) -> auto& { return get_environment(context); }, context);
}

} // namespace detail

} // namespace xronos::sdk

#endif // XRONOS_SDK_CONTEXT_HH
