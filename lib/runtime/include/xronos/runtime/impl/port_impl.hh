// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_IMPL_PORT_IMPL_HH
#define XRONOS_RUNTIME_IMPL_PORT_IMPL_HH

#include "xronos/runtime/assert.hh"
#include "xronos/runtime/connection.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/port.hh"
#include "xronos/runtime/reactor.hh"

namespace xronos::runtime {

// connection fwd

template <class T> class Connection;
template <class T> class DelayedConnection;
template <class T> class PhysicalConnection;
template <class T> class EnclaveConnection;
template <class T> class DelayedEnclaveConnection;
template <class T> class PhysicalEnclaveConnection;

template <class T> [[maybe_unused]] auto Port<T>::typed_outward_bindings() const noexcept -> const std::set<Port<T>*>& {
  // HACK this cast is ugly but should be safe as long as we only allow to
  // bind with Port<T>*. The alternative would be to copy the entire set and
  // cast each element individually, which is also ugly...
  // TODO USE C++20 std::bit_cast
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return reinterpret_cast<const std::set<Port<T>*>&>(outward_bindings());
}

template <class T> auto Port<T>::typed_inward_binding() const noexcept -> Port<T>* {
  // we can use a static cast here since we know that this port is always
  // connected with another Port<T>.
  return static_cast<Port<T>*>(inward_binding());
}

template <class T> void Port<T>::set(const ImmutableValuePtr<T>& value_ptr) {
  validate(!has_inward_binding(), "set() may only be called on ports that do not have an inward "
                                  "binding!");
  validate(value_ptr != nullptr, "Ports may not be set to nullptr!");

  auto scheduler = environment().scheduler();
  this->value_ptr_ = std::move(value_ptr);
  scheduler->set_port(this);
  this->present_ = true;
}

template <class T> auto Port<T>::get() const noexcept -> const ImmutableValuePtr<T>& {
  if (has_inward_binding()) {
    return typed_inward_binding()->get();
  }
  return value_ptr_;
}
template <class T>
void Port<T>::instantiate_connection_to(const ConnectionProperties& properties,
                                        const std::vector<BasePort*>& downstream) {
  std::unique_ptr<Connection<T>> connection = nullptr;
  if (downstream.empty()) {
    return;
  }

  // normal connections should be handled by environment
  reactor_assert(properties.type_ != ConnectionType::Normal);

  Environment& enclave = downstream[0]->environment();
  auto index = this->container()->connections().size();

  if (properties.type_ == ConnectionType::Delayed) {
    connection = std::make_unique<DelayedConnection<T>>(this->name() + "_delayed_connection_" + std::to_string(index),
                                                        *this->container(), properties.delay_);
  }
  if (properties.type_ == ConnectionType::Physical) {
    connection = std::make_unique<PhysicalConnection<T>>(this->name() + "_physical_connection_" + std::to_string(index),
                                                         *this->container(), properties.delay_);
  }
  if (properties.type_ == ConnectionType::Enclaved) {
    connection =
        std::make_unique<EnclaveConnection<T>>(this->name() + "_enclave_connection_" + std::to_string(index), enclave);
  }
  if (properties.type_ == ConnectionType::DelayedEnclaved) {
    connection = std::make_unique<DelayedEnclaveConnection<T>>(
        this->name() + "_delayed_enclave_connection_" + std::to_string(index), enclave, properties.delay_);
  }
  if (properties.type_ == ConnectionType::PhysicalEnclaved) {
    connection = std::make_unique<PhysicalEnclaveConnection<T>>(
        this->name() + "_physical_enclave_connection_" + std::to_string(index), enclave);
  }

  // if the connection here is null we have a vaulty enum value
  reactor_assert(connection != nullptr);
  connection->bind_downstream_ports(downstream);
  connection->bind_upstream_port(this);
  this->register_set_callback(connection->upstream_set_callback());
  this->container()->add_connection(std::move(connection));
}

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_IMPL_PORT_IMPL_HH
