// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/backend/backend.hh"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/backend/engine.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::backend {

Backend::Backend(RuntimeFactory runtime_factory)
    : engine_{std::make_unique<Engine>()}
    , runtime_factory_{std::move(runtime_factory)} {}
Backend::~Backend() = default;
Backend::Backend(Backend&&) noexcept = default;
auto Backend::operator=(Backend&&) noexcept -> Backend& = default;

auto Backend::abi() const noexcept -> abi::Backend& { return engine_->abi(); }

void Backend::set_runtime_factory(RuntimeFactory runtime_factory) { runtime_factory_ = std::move(runtime_factory); }

void Backend::enable_telemetry(std::string_view application_name, std::string_view endpoint) {
  engine_->enable_telemetry(application_name, endpoint);
}

void Backend::assemble() { engine_->assemble(); }

auto Backend::validate() -> std::vector<std::string> { return engine_->validate(); }

void Backend::export_diagram() { engine_->export_diagram(); }

void Backend::run(const runtime::ExecutionProperties& properties) { engine_->run(runtime_factory_(), properties); }

} // namespace xronos::backend
