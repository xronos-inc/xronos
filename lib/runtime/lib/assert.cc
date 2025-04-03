// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/assert.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/reactor_element.hh"

namespace xronos::runtime {

void assert_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] Phase phase) {
  if constexpr (runtime_assertion) {
    if (ptr->environment().phase() != phase) {
      auto enum_value_to_name = [](Phase phase) -> std::string {
        const std::map<Phase, std::string> conversation_map = {
            {Phase::Construction, "Construction"}, {Phase::Assembly, "Assembly"},
            {Phase::Startup, "Startup"},           {Phase::Execution, "Execution"},
            {Phase::Shutdown, "Shutdown"},         {Phase::Deconstruction, "Deconstruction"}};
        // in C++20 use .contains()
        if (conversation_map.find(phase) != std::end(conversation_map)) {
          return conversation_map.at(phase);
        }
        return "Unknown Phase: Value: " + std::to_string(extract_value(phase));
      };
      print_backtrace();

      throw ValidationError("Expected Phase: " + enum_value_to_name(phase) +
                            " Current Phase: " + enum_value_to_name(ptr->environment().phase()));
    }
  }
}

} // namespace xronos::runtime
