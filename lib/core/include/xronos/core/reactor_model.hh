// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_REACTOR_MODEL_HH
#define XRONOS_CORE_REACTOR_MODEL_HH

#include "xronos/core/connection_graph.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reaction_dependency_registry.hh"

namespace xronos::core {

struct ReactorModel {
  ElementRegistry element_registry{};
  ConnectionGraph connection_graph{};
  ReactionDependencyRegistry reaction_dependency_registry{};
};

} // namespace xronos::core

#endif // XRONOS_CORE_REACTOR_MODEL_HH
