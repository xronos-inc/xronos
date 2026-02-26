// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/otel/otel_reaction_span_logger.hh"

#include <cstdint>
#include <memory>
#include <utility>

#include "common.hh"
#include "opentelemetry/trace/scope.h"
#include "opentelemetry/trace/span.h"
#include "xronos/core/element.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/telemetry/reaction.hh"
#include "xronos/util/assert.hh"

namespace xronos::telemetry::otel {

class OtelReactionSpanScope final : public ReactionSpanScope {
  opentelemetry::trace::Scope scope_;
  std::shared_ptr<opentelemetry::trace::Span> span_;

public:
  OtelReactionSpanScope(std::shared_ptr<opentelemetry::trace::Span> span)
      : scope_(opentelemetry::trace::Tracer::WithActiveSpan(span))
      , span_(std::move(span)) {};
  OtelReactionSpanScope(const OtelReactionSpanScope&) = delete;
  OtelReactionSpanScope(OtelReactionSpanScope&&) = default;
  auto operator=(const OtelReactionSpanScope&) = delete;
  auto operator=(OtelReactionSpanScope&&) -> OtelReactionSpanScope& = default;

  ~OtelReactionSpanScope() final { span_->End(); }
};

auto OtelReactionSpanLogger::record_reaction_span(std::uint64_t reaction_uid,
                                                  const runtime::ProgramHandle& program_handle)
    -> std::unique_ptr<ReactionSpanScope> {
  const auto& element = element_registry_.get().get(reaction_uid);
  auto attributes = get_low_cardinality_attributes(attribute_manager_, element_registry_, element);

  // Compute list of low cardinality attributes. This assumes that all
  // attributes set so far are low cardinality. We need to explicitly own this
  // list here until we set the attributes, as the attribute map only stores a
  // (non-owning) span.
  auto low_cardinality_attributes = get_attribute_names(attributes);
  attributes["xronos.schema.low_cardinality_attributes"] = low_cardinality_attributes;

  util::assert_(element.parent_uid.has_value());
  const runtime::TimeAccess& time_access = *program_handle.get_time_access(element.parent_uid.value());
  set_common_high_cardinality_attributes(time_access, attributes);

  auto deadline = core::get_properties<core::ReactionTag>(element).deadline;
  if (deadline.has_value()) {
    attributes["xronos.deadline"] = (time_access.get_timestamp().time_since_epoch() + *deadline).count();
  }

  auto tracer = get_tracer();
  auto span = tracer->StartSpan(element.fqn, attributes);
  return std::make_unique<OtelReactionSpanScope>(std::move(span));
}

} // namespace xronos::telemetry::otel
