// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DATA_LOGGER_HH
#define XRONOS_RUNTIME_DATA_LOGGER_HH

#include "fwd.hh"

#include <memory>

namespace xronos::runtime {

class RuntimeDataLogger {
public:
  virtual ~RuntimeDataLogger() = default;

  class ReactionScope {
  public:
    virtual ~ReactionScope() = default;
  };
  using ReactionScopePtr = std::unique_ptr<ReactionScope>;

  virtual auto record_reaction_start(const Reaction& reaction) -> ReactionScopePtr = 0;
};

class NoopRuntimeDataLogger : public RuntimeDataLogger {
public:
  auto record_reaction_start([[maybe_unused]] const Reaction& reaction) -> ReactionScopePtr final { return nullptr; }
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_DATA_LOGGER_HH
