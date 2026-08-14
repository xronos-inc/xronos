// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/validator/checks.hh"

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "nonstd/expected.hpp"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"

namespace xronos::validator {

namespace {

// Collects every port a subtree serializes or deserializes at that carries
// no serializer.
void collect_ports_without_serializer(const core::DeliverySubtree& subtree, const core::ReactorModel& model,
                                      std::set<core::ElementID>& ports_without_serializer) {
  auto require_serializer = [&](core::ElementID uid) {
    const auto& element = model.element_registry.get(uid);
    if (core::get_port_properties(element).serializer == nullptr) {
      ports_without_serializer.insert(uid);
    }
  };
  for (const auto& serialization : subtree.serializations) {
    require_serializer(serialization.serialize_uid);
    for (const auto& branch : serialization.branches) {
      require_serializer(branch.deserialize_uid);
      collect_ports_without_serializer(branch.subtree, model, ports_without_serializer);
    }
  }
}

} // namespace

// Checks that every port that is involved in cross-boundary communication
// defines a serializer.
auto check_cross_boundary_serializers(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>> {
  // A set keeps the messages free of duplicates when one port sits on
  // several crossings, and orders them by uid.
  std::set<core::ElementID> ports_without_serializer;
  for (const auto& tree : model.connection_graph.compute_delivery_trees()) {
    collect_ports_without_serializer(tree.root, model, ports_without_serializer);
  }

  std::vector<std::string> error_messages;
  error_messages.reserve(ports_without_serializer.size());
  for (auto uid : ports_without_serializer) {
    error_messages.push_back(fmt::format("Port {} is on a cross-boundary connection but has no serializer defined.",
                                         model.element_registry.get(uid).fqn));
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

} // namespace xronos::validator
