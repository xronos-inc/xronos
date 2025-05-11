#include "xronos/sdk/startup.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::sdk {

Startup::Startup(std::string_view name, ReactorContext context)
    : Element{std::make_unique<runtime::StartupTrigger>(name, detail::get_reactor_instance(context)), context} {}

[[nodiscard]] auto Startup::is_present() const noexcept -> bool {
  return detail::get_runtime_instance<runtime::StartupTrigger>(*this).is_present();
}

void Startup::register_as_trigger_of(runtime::Reaction& reaction) const noexcept {
  reaction.declare_trigger(&detail::get_runtime_instance<runtime::StartupTrigger>(*this));
}

} // namespace xronos::sdk
