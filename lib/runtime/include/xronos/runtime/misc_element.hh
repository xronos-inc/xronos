// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_MISC_ELEMENT_HH
#define XRONOS_RUNTIME_MISC_ELEMENT_HH

#include <string_view>

#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/reactor_element.hh"

namespace xronos::runtime {

// An element that is not directly relevant to the runtime. Can be used to
// create additional elements that behave similarly to the core runtime
// elements, but that do not impact the execution behavior.
class MiscElement : public ReactorElement {

public:
  MiscElement(std::string_view name, Reactor& container)
      : ReactorElement(name, container) {}

  void startup() override {}
  void shutdown() override {}

  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_MISC_ELEMENT_HH
