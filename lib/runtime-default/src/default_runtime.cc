// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/default_runtime.hh"

#include <memory>

#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/default/detail/default_runtime_impl.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_ {

auto DefaultRuntime::initialize_reactor_program(const core::ReactorModel& model, const ExecutionProperties& properties)
    -> std::unique_ptr<ProgramHandle> {
  auto handle = std::make_unique<detail::DefaultRuntimeImpl>(properties);
  handle->initialize(model);
  return handle;
}

} // namespace xronos::runtime::default_
