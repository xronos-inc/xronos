// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/reaction.hh"

#include "xronos/runtime/reaction.hh"
#include "xronos/runtime/reactor.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/reactor.hh"

namespace xronos::sdk {

BaseReaction::BaseReaction(ReactionProperties properties)
    : Element{detail::make_runtime_element_pointer<runtime::Reaction>(
                  properties.name_, properties.reaction_id_,
                  detail::get_runtime_instance<runtime::Reactor>(properties.container()), [this]() { handler(); }),
              properties.context_} {}

[[nodiscard]] auto BaseReaction::context() noexcept -> ReactionContext {
  return ReactionContext{detail::get_runtime_instance<runtime::Reaction>(*this)};
}

} // namespace xronos::sdk
