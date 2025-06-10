#include <functional>
#include <memory>
#include <source_location>
#include <string_view>

#include "xronos/sdk/element.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/reactor.hh"

#include "xronos/runtime/environment.hh"
#include "xronos/runtime/reactor.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/startup.hh"

namespace xronos::sdk {

class ReactorWrapper : public runtime::Reactor {
public:
  ReactorWrapper(std::string_view name, runtime::Reactor& container, std::function<void()> assemble_callback)
      : runtime::Reactor{name, container}
      , assemble_callback_{std::move(assemble_callback)} {}
  ReactorWrapper(std::string_view name, runtime::Environment& environment, std::function<void()> assemble_callback)
      : runtime::Reactor{name, environment}
      , assemble_callback_{std::move(assemble_callback)} {}

private:
  void assemble() final { assemble_callback_(); }
  std::function<void()> assemble_callback_;
};

namespace detail {

auto make_reactor_wrapper(std::string_view name, ReactorContext context, std::function<void()> assemble) {
  return std::make_unique<ReactorWrapper>(name, detail::get_reactor_instance(context), std::move(assemble));
}

auto make_reactor_wrapper(std::string_view name, EnvironmentContext context, std::function<void()> assemble) {
  return std::make_unique<ReactorWrapper>(name, detail::get_environment_instance(detail::get_environment(context)),
                                          std::move(assemble));
}

} // namespace detail

Reactor::Reactor(std::string_view name, Context parent_context)
    : Element{std::visit(
                  [name, this](auto ctx) { return detail::make_reactor_wrapper(name, ctx, [this]() { assemble(); }); },
                  parent_context),
              parent_context}
    , environment_{std::visit([](auto ctx) -> Environment& { return detail::get_environment(ctx); }, parent_context)}
    , startup_{"startup", this->context()}
    , shutdown_{"shutdown", this->context()} {}

[[nodiscard]] auto Reactor::context(std::source_location source_location) noexcept -> ReactorContext {
  return detail::create_context(*this, detail::SourceLocationView::from_std(source_location));
}

auto Reactor::get_time() const noexcept -> TimePoint {
  return detail::get_runtime_instance<runtime::Reactor>(*this).get_logical_time();
}

auto Reactor::get_lag() const noexcept -> Duration { return std::chrono::system_clock::now() - get_time(); }

auto Reactor::get_time_since_startup() const noexcept -> Duration {
  return detail::get_runtime_instance<runtime::Reactor>(*this).get_elapsed_logical_time();
}

void Reactor::request_shutdown() noexcept {
  detail::get_runtime_instance<runtime::Reactor>(*this).environment().sync_shutdown();
}

} // namespace xronos::sdk
