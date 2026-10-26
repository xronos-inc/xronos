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
 * The environment's context, through which it creates top-level reactors.
 *
 * Use Environment::context() to obtain an instance of this class.
 */
class EnvironmentContext {
private:
  EnvironmentContext(const std::shared_ptr<detail::ProgramContext>& program_context,
                     detail::SourceLocationView source_location)
      : program_context_{program_context}
      , source_location_{source_location} {}
  // A reference wrapper keeps the context trivially copyable; it refers to the
  // stable program_context_ the Environment owns.
  std::reference_wrapper<const std::shared_ptr<detail::ProgramContext>> program_context_;
  detail::SourceLocationView source_location_;

  friend detail::ContextAccess;
};

/**
 * A reactor's context, through which it creates its @ref Element "elements".
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

  std::reference_wrapper<const std::shared_ptr<detail::ProgramContext>> program_context_;
  std::uint64_t parent_uid_;
  detail::SourceLocationView source_location_;

  friend detail::ContextAccess;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_CONTEXT_HH
