// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DEFAULT_RUNTIME_HH
#define XRONOS_RUNTIME_DEFAULT_DEFAULT_RUNTIME_HH

#include <memory>

#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_ {

class DefaultRuntime final : public Runtime {
public:
  [[nodiscard]] auto initialize_reactor_program(const core::ReactorModel& model, const ExecutionProperties& properties)
      -> std::unique_ptr<ProgramHandle> final;
};

} // namespace xronos::runtime::default_

#endif // XRONOS_RUNTIME_DEFAULT_DEFAULT_RUNTIME_HH
