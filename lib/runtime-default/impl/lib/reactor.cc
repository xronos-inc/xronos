// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/impl/reactor.hh"

#include <cstdint>
#include <memory>
#include <string_view>
#include <utility>

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/assert.hh"
#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/logical_time.hh"
#include "xronos/runtime/default/impl/reaction.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::runtime::default_::impl {

Reactor::Reactor(const core::Element& element_info, Reactor& container)
    : ReactorElement(element_info, container) {
  container.register_contained_reactor(*this);
}
Reactor::Reactor(const core::Element& element_info, Environment& environment)
    : ReactorElement(element_info, environment) {
  environment.register_top_level_reactor(*this);
}

void Reactor::register_element(ReactorElement& element) {
  validate(this == element.container(), "Only elements contained by the reactor can be registered");
  validate(this->environment().phase() == Phase::Construction || this->environment().phase() == Phase::Assembly,
           "Elements can only be registered during construction phase!");
  [[maybe_unused]] bool result = elements_.insert(&element).second;
  util::assert_(result);
}

void Reactor::register_contained_reactor(Reactor& reactor) {
  validate(elements_.contains(&reactor), "Cannot register a reactor that is not registered as an element");
  contained_reactors_.push_back(&reactor);
}

void Reactor::register_reaction(Reaction& reaction) {
  validate(elements_.contains(&reaction), "Cannot register a reaction that is not registered as an element");
  reactions_.push_back(&reaction);
}

void Reactor::add_connection([[maybe_unused]] std::unique_ptr<BaseAction> connection) {
  util::assert_(connection != nullptr);
  connections_.emplace_back(std::move(connection));
}

void Reactor::startup() {
  util::assert_(environment().phase() == Phase::Startup);
  util::log::debug() << "Starting up reactor " << fqn();
  // call startup on all contained objects
  for (auto* element : elements_) {
    element->startup();
  }
}

void Reactor::shutdown() {
  util::assert_(environment().phase() == Phase::Shutdown);
  util::log::debug() << "Terminating reactor " << fqn();
  // call shutdown on all contained objects
  for (auto* element : elements_) {
    element->shutdown();
  }
}

auto Reactor::get_physical_time() noexcept -> TimePoint { return impl::get_physical_time(); }

auto Reactor::get_logical_time() const noexcept -> TimePoint {
  return environment().scheduler()->logical_time().time_point();
}

auto Reactor::get_microstep() const noexcept -> std::uint32_t {
  return environment().scheduler()->logical_time().micro_step();
}

auto Reactor::get_tag() const noexcept -> Tag {
  return Tag::from_logical_time(environment().scheduler()->logical_time());
}

auto Reactor::get_elapsed_logical_time() const noexcept -> Duration {
  return get_logical_time() - environment().start_tag().time_point();
}

auto Reactor::get_elapsed_physical_time() const noexcept -> Duration {
  return get_physical_time() - environment().start_tag().time_point();
}

} // namespace xronos::runtime::default_::impl
