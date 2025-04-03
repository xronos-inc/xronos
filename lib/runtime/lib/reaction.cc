// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include <cstdint>
#include <utility>

#include "xronos/runtime/reaction.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/assert.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/port.hh"

namespace xronos::runtime {

Reaction::Reaction(std::string_view name, std::uint32_t priority, Reactor& container, std::function<void(void)> body)
    : ReactorElement(name, container)
    , priority_(priority)
    , body_(std::move(std::move(body))) {
  reactor_assert(priority != 0);
  container.register_reaction(*this);
}

void Reaction::declare_trigger(BaseAction* action) {
  reactor_assert(action != nullptr);
  reactor_assert(&this->environment() == &action->environment());
  assert_phase(this, Phase::Assembly);
  validate(this->container() == action->container(), "Action triggers must belong to the same reactor as the triggered "
                                                     "reaction");

  [[maybe_unused]] bool result = action_triggers_.insert(action).second;
  reactor_assert(result);
  action->register_trigger(this);
}

void Reaction::declare_schedulable_action(BaseAction* action) {
  reactor_assert(action != nullptr);
  reactor_assert(&this->environment() == &action->environment());
  assert_phase(this, Phase::Assembly);
  validate(this->container() == action->container(), "Scheduable actions must belong to the same reactor as the "
                                                     "triggered reaction");

  [[maybe_unused]] bool result = scheduable_actions_.insert(action).second;
  reactor_assert(result);
  action->register_scheduler(this);
}

void Reaction::declare_trigger(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(&this->environment() == &port->environment());
  assert_phase(this, Phase::Assembly);

  if (port->is_input()) {
    validate(this->container() == port->container(),
             "Input port triggers must belong to the same reactor as the triggered "
             "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Output port triggers must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = port_trigger_.insert(port).second;
  reactor_assert(result);
  result = dependencies_.insert(port).second;
  reactor_assert(result);
  port->register_dependency(this, true);
}

void Reaction::declare_dependency(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(&this->environment() == &port->environment());
  assert_phase(this, Phase::Assembly);

  if (port->is_input()) {
    validate(this->container() == port->container(), "Dependent input ports must belong to the same reactor as the "
                                                     "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = dependencies_.insert(port).second;
  reactor_assert(result);
  port->register_dependency(this, false);
}

void Reaction::declare_dependency(BaseAction* action) {
  reactor_assert(action != nullptr);
  reactor_assert(&this->environment() == &action->environment());
  assert_phase(this, Phase::Assembly);

  validate(this->container() == action->container(), "Source actions must belong to the same reactor as the "
                                                     "reaction");

  [[maybe_unused]] bool result = action_dependencies_.insert(action).second;
  reactor_assert(result);
}

void Reaction::declare_antidependency(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(&this->environment() == &port->environment());
  assert_phase(this, Phase::Assembly);

  if (port->is_output()) {
    validate(this->container() == port->container(), "Antidependent output ports must belong to the same reactor as "
                                                     "the reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = antidependencies_.insert(port).second;
  reactor_assert(result);
  port->register_antidependency(this);
}

void Reaction::trigger() {
  if (has_deadline()) {
    reactor_assert(deadline_handler_ != nullptr);
    auto lag = Reactor::get_physical_time() - container()->get_logical_time();
    if (lag > deadline_) {
      deadline_handler_();
      return;
    }
  }

  body_();
}

void Reaction::set_deadline_impl(Duration deadline, const std::function<void(void)>& handler) {
  reactor_assert(!has_deadline());
  reactor_assert(handler != nullptr);
  this->deadline_ = deadline;
  this->deadline_handler_ = handler;
}

void Reaction::set_index(unsigned index) {
  validate(this->environment().phase() == Phase::Assembly, "Reaction indexes may only be set during assembly phase!");
  this->index_ = index;
}

} // namespace xronos::runtime
