// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DETAIL_RUNTIME_MODEL_HH
#define XRONOS_RUNTIME_DEFAULT_DETAIL_RUNTIME_MODEL_HH

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"

namespace xronos::runtime::default_::detail {

struct TriggerProperties {
  std::vector<std::uint64_t> triggered_reaction_uids;
};

// The core's delivery trees, mirrored with the serializers resolved from
// the port properties and pruned to the ports that trigger reactions. The
// uids stay alongside the resolved serializers for error messages.

struct ResolvedBoundarySerialization;

struct ResolvedDeliverySubtree {
  std::vector<core::DeliveryLeaf> leaves;
  std::vector<ResolvedBoundarySerialization> serializations;
};

struct ResolvedBoundaryDeserialization {
  abi::PortSerializer* deserializer{};
  std::uint64_t deserialize_uid{};
  ResolvedDeliverySubtree subtree;
};

struct ResolvedBoundarySerialization {
  abi::PortSerializer* serializer{};
  std::uint64_t serialize_uid{};
  std::vector<ResolvedBoundaryDeserialization> branches;
};

class RuntimeModel {
public:
  void init(const core::ReactorModel& model);

  std::unordered_map<std::uint64_t, TriggerProperties> triggers;
  std::vector<std::uint64_t> startup_trigger_uids;
  std::vector<std::uint64_t> shutdown_trigger_uids;
  std::unordered_map<std::uint64_t, core::PeriodicTimerProperties> periodic_timer_properties;
  std::unordered_map<std::uint64_t, ResolvedDeliverySubtree> delivery_trees;
  std::vector<std::uint64_t> ordered_reaction_uids;
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_RUNTIME_MODEL_HH
