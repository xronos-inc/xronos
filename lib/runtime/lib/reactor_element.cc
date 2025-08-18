// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/reactor.hh"

#include <sstream>
#include <string_view>

#include "xronos/runtime/environment.hh"
#include "xronos/runtime/reactor_element.hh"

namespace xronos::runtime {

ReactorElement::ReactorElement(std::string_view name, Reactor& container)
    : name_(name)
    , fqn_(compute_fqn(container, name))
    , uid_(ReactorElement::generate_uid())
    , container_(&container)
    , environment_(container.environment()) {
  container.register_element(*this);
}

ReactorElement::ReactorElement(std::string_view name, Environment& environment)
    : name_(name)
    , fqn_(name)
    , uid_(ReactorElement::generate_uid())
    , environment_(environment) {}

auto ReactorElement::compute_fqn(const Reactor& container, std::string_view name) -> std::string {
  std::stringstream sstream;
  sstream << container.fqn() << '.' << name;
  return sstream.str();
}

} // namespace xronos::runtime
