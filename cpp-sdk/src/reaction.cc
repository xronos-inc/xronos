#include <memory>

#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/reactor.hh"

#include "xronos/runtime/reaction.hh"
#include "xronos/runtime/reactor.hh"

namespace xronos::sdk {

BaseReaction::BaseReaction(ReactionProperties properties)
    : Element{std::make_unique<runtime::Reaction>(
                  properties.name_, properties.reaction_id_,
                  detail::get_runtime_instance<runtime::Reactor>(properties.container()), [this]() { handler(); }),
              properties.context_} {}

[[nodiscard]] auto BaseReaction::context() noexcept -> ReactionContext {
  return ReactionContext{detail::get_runtime_instance<runtime::Reaction>(*this)};
}

} // namespace xronos::sdk
