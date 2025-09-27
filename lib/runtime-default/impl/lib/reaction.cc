// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/impl/reaction.hh"

#include <functional>
#include <string_view>
#include <utility>

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/assert.hh"
#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/port.hh"
#include "xronos/runtime/default/impl/reactor.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/util/assert.hh"

namespace xronos::runtime::default_::impl {

Reaction::Reaction(const core::Element& element_info, Reactor& container)
    : ReactorElement(element_info, container)
    , reaction_properties_(*std::get<core::ReactionTag>(element_info.type).properties) {
  container.register_reaction(*this);
}

void Reaction::declare_trigger(BaseAction* action) {
  util::assert_(action != nullptr);
  util::assert_(&this->environment() == &action->environment());
  validate_phase(this, Phase::Construction);
  validate(this->container() == action->container(), "Action triggers must belong to the same reactor as the triggered "
                                                     "reaction");

  [[maybe_unused]] bool result = action_triggers_.insert(action).second;
  util::assert_(result);
  action->register_trigger(this);
}

void Reaction::declare_schedulable_action(BaseAction* action) {
  util::assert_(action != nullptr);
  util::assert_(&this->environment() == &action->environment());
  validate_phase(this, Phase::Construction);
  validate(this->container() == action->container(), "Scheduable actions must belong to the same reactor as the "
                                                     "triggered reaction");

  [[maybe_unused]] bool result = scheduable_actions_.insert(action).second;
  util::assert_(result);
  action->register_scheduler(this);
}

void Reaction::declare_trigger(Port* port) {
  util::assert_(port != nullptr);
  util::assert_(&this->environment() == &port->environment());
  validate_phase(this, Phase::Construction);

  if (port->is_input()) {
    validate(this->container() == port->container(),
             "Input port triggers must belong to the same reactor as the triggered "
             "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Output port triggers must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = port_trigger_.insert(port).second;
  util::assert_(result);
  result = dependencies_.insert(port).second;
  util::assert_(result);
  port->register_dependency(this, true);
}

void Reaction::declare_dependency(Port* port) {
  util::assert_(port != nullptr);
  util::assert_(&this->environment() == &port->environment());
  validate_phase(this, Phase::Construction);

  if (port->is_input()) {
    validate(this->container() == port->container(), "Dependent input ports must belong to the same reactor as the "
                                                     "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = dependencies_.insert(port).second;
  util::assert_(result);
  port->register_dependency(this, false);
}

void Reaction::declare_dependency(BaseAction* action) {
  util::assert_(action != nullptr);
  util::assert_(&this->environment() == &action->environment());
  validate_phase(this, Phase::Construction);

  validate(this->container() == action->container(), "Source actions must belong to the same reactor as the "
                                                     "reaction");

  [[maybe_unused]] bool result = action_dependencies_.insert(action).second;
  util::assert_(result);
}

void Reaction::declare_antidependency(Port* port) {
  util::assert_(port != nullptr);
  util::assert_(&this->environment() == &port->environment());
  validate_phase(this, Phase::Construction);

  if (port->is_output()) {
    validate(this->container() == port->container(), "Antidependent output ports must belong to the same reactor as "
                                                     "the reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = antidependencies_.insert(port).second;
  util::assert_(result);
  port->register_antidependency(this);
}

void Reaction::trigger() {
  if (has_deadline()) {
    util::assert_(deadline_handler_ != nullptr);
    auto lag = Reactor::get_physical_time() - container()->get_logical_time();
    if (lag > deadline_) {
      deadline_handler_();
      return;
    }
  }

  reaction_properties_.get().handler();
}

void Reaction::set_deadline_impl(Duration deadline, const std::function<void(void)>& handler) {
  util::assert_(!has_deadline());
  util::assert_(handler != nullptr);
  this->deadline_ = deadline;
  this->deadline_handler_ = handler;
}

void Reaction::set_index(unsigned index) {
  validate(this->environment().phase() == Phase::Assembly, "Reaction indexes may only be set during assembly phase!");
  this->index_ = index;
}

} // namespace xronos::runtime::default_::impl
