// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_DETAIL_ELEMENT_HH
#define XRONOS_SDK_DETAIL_ELEMENT_HH

#include <cstdint>
#include <string>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk::detail {

[[nodiscard]] inline auto get_backend(const Context& context) noexcept -> abi::Backend& {
  return ContextAccess::get_program_context(context)->backend();
}

// Runs a Backend::register_* call and decorates an InvalidNameError with the
// element's source location (the implementation cannot know it).
template <class Fn> auto register_with_location(const Context& context, Fn&& register_fn) -> std::uint64_t {
  try {
    return std::forward<Fn>(register_fn)();
  } catch (const InvalidNameError& error) {
    auto source_location = ContextAccess::get_source_location(context);
    throw InvalidNameError(std::string{source_location.file} + ":" + std::to_string(source_location.start_line) + ": " +
                           error.what());
  }
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_DETAIL_ELEMENT_HH
