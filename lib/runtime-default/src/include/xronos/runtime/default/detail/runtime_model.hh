// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DETAIL_RUNTIME_MODEL_HH
#define XRONOS_RUNTIME_DEFAULT_DETAIL_RUNTIME_MODEL_HH

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"

namespace xronos::runtime::default_::detail {

struct TriggerProperties {
  std::vector<std::uint64_t> triggered_reaction_uids;
};

class RuntimeModel {
public:
  void init(const core::ReactorModel& model);

  std::unordered_map<std::uint64_t, TriggerProperties> triggers;
  std::vector<std::uint64_t> startup_trigger_uids;
  std::vector<std::uint64_t> shutdown_trigger_uids;
  std::unordered_map<std::uint64_t, core::PeriodicTimerProperties> periodic_timer_properties;
  std::unordered_map<std::uint64_t, std::vector<core::ConnectionProperties>> end_to_end_connections;
  std::vector<std::uint64_t> ordered_reaction_uids;
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_RUNTIME_MODEL_HH
