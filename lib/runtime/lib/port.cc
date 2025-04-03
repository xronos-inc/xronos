// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/port.hh"

#include "xronos/runtime/assert.hh"
#include "xronos/runtime/connection.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::runtime {

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) noexcept {
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

void BasePort::register_antidependency(Reaction* reaction) noexcept {
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

[[maybe_unused]] auto Port<void>::typed_outward_bindings() const noexcept -> const std::set<Port<void>*>& {
  // this is undefined behavior and should be changed
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return reinterpret_cast<const std::set<Port<void>*>&>(outward_bindings()); // use C++20 std::bit_cast
}

auto Port<void>::typed_inward_binding() const noexcept -> Port<void>* {
  // we can use a reinterpret cast here since we know that this port is always
  // connected with another Port<T>.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return reinterpret_cast<Port<void>*>(inward_binding());
}

void Port<void>::set() {
  validate(!has_inward_binding(), "set() may only be called on ports that do not have an inward "
                                  "binding!");

  auto* scheduler = environment().scheduler();
  scheduler->set_port(this);
  this->present_ = true;
}

void Port<void>::instantiate_connection_to(const ConnectionProperties& properties,
                                           const std::vector<BasePort*>& downstream) {
  std::unique_ptr<Connection<void>> connection = nullptr;

  if (downstream.empty()) {
    return;
  }

  // normal connections should be handled by environment
  reactor_assert(properties.type_ != ConnectionType::Normal);

  Environment& enclave = downstream[0]->environment();
  auto index = this->container()->connections().size();

  if (properties.type_ == ConnectionType::Delayed) {
    connection = std::make_unique<DelayedConnection<void>>(
        this->name() + "_delayed_connection_" + std::to_string(index), *this->container(), properties.delay_);
  }
  if (properties.type_ == ConnectionType::Physical) {
    connection = std::make_unique<PhysicalConnection<void>>(
        this->name() + "_physical_connection_" + std::to_string(index), *this->container(), properties.delay_);
  }
  if (properties.type_ == ConnectionType::Enclaved) {
    connection = std::make_unique<EnclaveConnection<void>>(
        this->name() + "_enclave_connection_" + std::to_string(index), enclave);
  }
  if (properties.type_ == ConnectionType::DelayedEnclaved) {
    connection = std::make_unique<DelayedEnclaveConnection<void>>(
        this->name() + "_delayed_enclave_connection_" + std::to_string(index), enclave, properties.delay_);
  }
  if (properties.type_ == ConnectionType::PhysicalEnclaved) {
    connection = std::make_unique<PhysicalEnclaveConnection<void>>(
        this->name() + "_physical_enclave_connection_" + std::to_string(index), enclave);
  }

  // if the connection here is null we have a vaulty enum value
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
  return [=](const BasePort& port) {
    callback1(port);
    callback2(port);
  };
}

void BasePort::register_set_callback(const PortCallback& callback) {
  if (set_callback_ == nullptr) {
    set_callback_ = callback;
  } else {
    set_callback_ = compose_callbacks(set_callback_, callback);
  }
}

void BasePort::register_clean_callback(const PortCallback& callback) {
  if (clean_callback_ == nullptr) {
    clean_callback_ = callback;
  } else {
    clean_callback_ = compose_callbacks(clean_callback_, callback);
  }
}

} // namespace xronos::runtime
