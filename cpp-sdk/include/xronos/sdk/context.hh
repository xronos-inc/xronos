// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_CONTEXT_HH
#define XRONOS_SDK_CONTEXT_HH

#include <cstdint>
#include <functional>
#include <memory>
#include <variant>

#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

class EnvironmentContext;
class ReactorContext;

/**
 * Can either hold an EnvironmentContext or a ReactorContext.
 */
using Context = std::variant<EnvironmentContext, ReactorContext>;

/**
 * Opaque data type used for constructing reactors.
 *
 * Use Environment::context() to obtain an instance of this class.
 */
class EnvironmentContext {
private:
  EnvironmentContext(const std::shared_ptr<detail::ProgramContext>& program_context,
                     detail::SourceLocationView source_location)
      : program_context_{program_context}
      , source_location_{source_location} {}
  // using a reference wrapper here to allow passing around context objects more
  // easily (without requiring std::move)
  std::reference_wrapper<const std::shared_ptr<detail::ProgramContext>> program_context_;
  detail::SourceLocationView source_location_;

  friend detail::ContextAccess;
};

/**
 * Opaque data type used for constructing @ref Element "Elements".
 *
 * Use Reactor::context() to obtain an instance of this class.
 */
class ReactorContext {
private:
  ReactorContext(const std::shared_ptr<detail::ProgramContext>& program_context, std::uint64_t reactor_uid,
                 detail::SourceLocationView source_location)
      : program_context_{program_context}
      , parent_uid_{reactor_uid}
      , source_location_{source_location} {}

  // using a reference wrapper here to allow passing around context objects more
  // easily (without requiring std::move)
  std::reference_wrapper<const std::shared_ptr<detail::ProgramContext>> program_context_;
  std::uint64_t parent_uid_;
  detail::SourceLocationView source_location_;

  friend detail::ContextAccess;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_CONTEXT_HH
