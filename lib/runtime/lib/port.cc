// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/port.hh"

#include <any>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "xronos/runtime/assert.hh"
#include "xronos/runtime/connection.hh"
#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::runtime {

void Port::register_dependency(Reaction* reaction, bool is_trigger) noexcept {
  reactor_assert(reaction != nullptr);
  reactor_assert(&this->environment() == &reaction->environment());
  validate(!this->has_outward_bindings(), "Dependencies may no be declared on ports with an outward binding!");
  assert_phase(this, Phase::Assembly);

  if (this->is_input()) {
    validate(this->container() == reaction->container(), "Dependent input ports must belong to the same reactor as the "
                                                         "reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = dependencies_.insert(reaction).second;
  reactor_assert(result);
  if (is_trigger) {
    result = triggers_.insert(reaction).second;
    reactor_assert(result);
  }
}

void Port::register_antidependency(Reaction* reaction) {
  reactor_assert(reaction != nullptr);
  reactor_assert(&this->environment() == &reaction->environment());
  validate(!this->has_inward_binding(), "Antidependencies may no be declared on ports with an inward binding!");
  assert_phase(this, Phase::Assembly);

  if (this->is_output()) {
    validate(this->container() == reaction->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = anti_dependencies_.insert(reaction).second;
  reactor_assert(result);
}

void Port::set(const std::any& value) {
  reactor_assert(!has_inward_binding());
  reactor_assert(value.has_value());

  auto* scheduler = environment().scheduler();
  current_value_ = value;
  scheduler->set_port(this);
}

auto Port::get() const noexcept -> const std::any& {
  if (has_inward_binding()) {
    return inward_binding()->get();
  }
  reactor_assert(current_value_.has_value());
  return current_value_;
}

auto Port::is_present() const noexcept -> bool {
  if (has_inward_binding()) {
    return inward_binding()->is_present();
  }
  return current_value_.has_value();
};

void Port::instantiate_connection_to(const ConnectionProperties& properties, const std::vector<Port*>& downstream) {
  std::unique_ptr<Connection> connection = nullptr;

  if (downstream.empty()) {
    return;
  }

  // normal connections should be handled by environment
  reactor_assert(properties.type_ != ConnectionType::Normal);

  auto index = this->container()->connections().size();

  if (properties.type_ == ConnectionType::Delayed) {
    connection = std::make_unique<DelayedConnection>(this->name() + "_delayed_connection_" + std::to_string(index),
                                                     *this->container(), properties.delay_);
  }
  if (properties.type_ == ConnectionType::Physical) {
    connection = std::make_unique<PhysicalConnection>(this->name() + "_physical_connection_" + std::to_string(index),
                                                      *this->container(), properties.delay_);
  }

  // if the connection here is null we have a faulty enum value
  reactor_assert(connection != nullptr);
  connection->bind_downstream_ports(downstream);
  connection->bind_upstream_port(this);
  this->register_set_callback(connection->upstream_set_callback());
  this->container()->add_connection(std::move(connection));
}

// This function can be used to chain two callbacks. This mechanism is not
// very efficient if many callbacks are registered on the same port. However,
// it is more efficient than, e.g., a vector of callbacks if usually only one
// callback is registered. At the moment, we use at most two callbacks on the
// same port (one if the port is in a multiport, and one if it is upstream of
// a delayed connection).
auto compose_callbacks(const PortCallback& callback1, const PortCallback& callback2) -> PortCallback {
  return [=](const Port& port) {
    callback1(port);
    callback2(port);
  };
}

void Port::invoke_set_callback() {
  if (set_callback_ != nullptr) {
    set_callback_(*this);
  }
}

void Port::invoke_clean_callback() {
  if (clean_callback_ != nullptr) {
    clean_callback_(*this);
  }
}

void Port::register_set_callback(const PortCallback& callback) {
  if (set_callback_ == nullptr) {
    set_callback_ = callback;
  } else {
    set_callback_ = compose_callbacks(set_callback_, callback);
  }
}

void Port::register_clean_callback(const PortCallback& callback) {
  if (clean_callback_ == nullptr) {
    clean_callback_ = callback;
  } else {
    clean_callback_ = compose_callbacks(clean_callback_, callback);
  }
}

} // namespace xronos::runtime
