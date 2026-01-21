// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_REACTION_HH
#define XRONOS_TELEMETRY_REACTION_HH

#include <cstdint>
#include <memory>

#include "xronos/runtime/interfaces.hh"

namespace xronos::telemetry {

class ReactionSpanScope {
public:
  virtual ~ReactionSpanScope() = default;
};

class ReactionSpanLogger {
public:
  virtual ~ReactionSpanLogger() = default;

  virtual auto record_reaction_span(std::uint64_t reaction_uid, const runtime::ProgramHandle& program_handle)
      -> std::unique_ptr<ReactionSpanScope> = 0;
};

class NoopReactionSpanLogger : public ReactionSpanLogger {
public:
  auto record_reaction_span([[maybe_unused]] std::uint64_t reaction_uid,
                            [[maybe_unused]] const runtime::ProgramHandle& program_handle)
      -> std::unique_ptr<ReactionSpanScope> final {
    return nullptr;
  }
};

} // namespace xronos::telemetry

#endif // XRONOS_TELEMETRY_REACTION_HH
