// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_OTEL_OTEL_REACTION_SPAN_LOGGER_HH
#define XRONOS_TELEMETRY_OTEL_OTEL_REACTION_SPAN_LOGGER_HH

#include <cstdint>
#include <functional>
#include <memory>

#include "xronos/core/element_registry.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/reaction.hh"

namespace xronos::telemetry::otel {

class OtelReactionSpanLogger : public ReactionSpanLogger {

public:
  OtelReactionSpanLogger(const AttributeManager& attribute_manager, const core::ElementRegistry& element_registry)
      : attribute_manager_(attribute_manager)
      , element_registry_(element_registry) {}

  auto record_reaction_span(std::uint64_t reaction_uid, const runtime::ProgramHandle& program_handle)
      -> std::unique_ptr<ReactionSpanScope> final;

private:
  std::reference_wrapper<const AttributeManager> attribute_manager_;
  std::reference_wrapper<const core::ElementRegistry> element_registry_;
};

} // namespace xronos::telemetry::otel

#endif // XRONOS_TELEMETRY_OTEL_OTEL_REACTION_SPAN_LOGGER_HH
