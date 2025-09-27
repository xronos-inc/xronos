// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/reactor.hh"

#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"

namespace xronos::runtime::default_::impl {

ReactorElement::ReactorElement(const core::Element& element_info, Reactor& container)
    : name_(element_info.name)
    , fqn_(element_info.fqn)
    , uid_(element_info.uid)
    , container_(&container)
    , environment_(container.environment()) {
  container.register_element(*this);
}

ReactorElement::ReactorElement(const core::Element& element_info, Environment& environment)
    : name_(element_info.name)
    , fqn_(element_info.fqn)
    , uid_(element_info.uid)
    , environment_(environment) {}

} // namespace xronos::runtime::default_::impl
