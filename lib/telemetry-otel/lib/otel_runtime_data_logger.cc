// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/otel/otel_runtime_data_logger.hh"

#include "common.hh"
#include "xronos/runtime/data_logger.hh"
#include "xronos/runtime/reaction.hh"

using namespace xronos::telemetry::otel;
using namespace xronos::runtime;

class OtelReactionScope final : public RuntimeDataLogger::ReactionScope {
  opentelemetry::trace::Scope scope_;
  std::shared_ptr<opentelemetry::trace::Span> span_;

public:
  OtelReactionScope(std::shared_ptr<opentelemetry::trace::Span> span)
      : scope_(get_tracer()->WithActiveSpan(span))
      , span_(std::move(span)) {};
  OtelReactionScope(const OtelReactionScope&) = delete;
  OtelReactionScope(OtelReactionScope&&) = default;
  auto operator=(const OtelReactionScope&) = delete;
  auto operator=(OtelReactionScope&&) = delete;

  ~OtelReactionScope() final { span_->End(); }
};

auto OtelRuntimeDataLogger::record_reaction_start(const Reaction& reaction) -> RuntimeDataLogger::ReactionScopePtr {
  auto attributes = get_low_cardinality_attributes(attribute_manager_, reaction);

  // Compute list of low cardinality attributes. This assumes that all
  // attributes set so far are low cardinality. We need to explicitly own this
  // list here until we set the attribtues, as the attribute map only stores a
  // (non-owning) span.
  auto low_cardinality_attributes = get_attribute_names(attributes);
  attributes["xronos.schema.low_cardinality_attributes"] = low_cardinality_attributes;

  set_common_high_cardinality_attributes(reaction, attributes);

  auto tracer = get_tracer();
  auto span = tracer->StartSpan(reaction.fqn(), attributes);
  return std::make_unique<OtelReactionScope>(std::move(span));
}
