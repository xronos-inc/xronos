// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/element.hh"

#include <string_view>
#include <variant>

#include "xronos/util/visitor.hh"

namespace xronos::core {

auto get_port_properties(const Element& element) -> PortProperties& {
  if (std::holds_alternative<InputPortTag>(element.type)) {
    return get_properties<InputPortTag>(element);
  }
  return get_properties<OutputPortTag>(element);
}

auto element_type_as_string(const ElementType& type) -> std::string_view {
  return std::visit(util::Visitor{
                        []([[maybe_unused]] const MetricTag&) { return "metric"; },
                        []([[maybe_unused]] const PeriodicTimerTag&) { return "periodic timer"; },
                        []([[maybe_unused]] const PhysicalEventTag&) { return "physical event"; },
                        []([[maybe_unused]] const InputPortTag&) { return "input port"; },
                        []([[maybe_unused]] const OutputPortTag&) { return "output port"; },
                        []([[maybe_unused]] const ProgrammableTimerTag&) { return "programmable timer"; },
                        []([[maybe_unused]] const ReactionTag&) { return "reaction"; },
                        []([[maybe_unused]] const ReactorTag&) { return "reactor"; },
                        []([[maybe_unused]] const ShutdownTag&) { return "shutdown trigger"; },
                        []([[maybe_unused]] const StartupTag&) { return "startup trigger"; },
                    },
                    type);
};

} // namespace xronos::core
