// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IMPL_XRONOS_SDK_DETAIL_CONTEXT_ACCESS_HH
#define IMPL_XRONOS_SDK_DETAIL_CONTEXT_ACCESS_HH

#include <cstdint>
#include <memory>
#include <optional>
#include <variant>

#include "program_context.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk::detail {

struct ContextAccess {
  [[nodiscard]] static auto create_environment_context(const std::shared_ptr<ProgramContext>& program_context,
                                                       SourceLocationView source_location) noexcept
      -> EnvironmentContext {
    return {program_context, source_location};
  }
  [[nodiscard]] static auto create_reactor_context(const std::shared_ptr<ProgramContext>& program_context,
                                                   std::uint64_t reactor_uid,
                                                   SourceLocationView source_location) noexcept -> ReactorContext {
    return {program_context, reactor_uid, source_location};
  }

  [[nodiscard]] static auto get_parent_uid(const Context& context) noexcept -> std::optional<std::uint64_t> {
    return std::visit([](const auto& context) -> std::optional<std::uint64_t> { return get_parent_uid(context); },
                      context);
  }

  [[nodiscard]] static auto get_parent_uid([[maybe_unused]] const EnvironmentContext& context) noexcept
      -> std::nullopt_t {
    return std::nullopt;
  }

  [[nodiscard]] static auto get_parent_uid(const ReactorContext& context) noexcept -> std::uint64_t {
    return context.parent_uid_;
  }

  [[nodiscard]] static auto get_program_context(const EnvironmentContext& context) noexcept
      -> const std::shared_ptr<ProgramContext>& {
    return context.program_context_;
  }
  [[nodiscard]] static auto get_program_context(const ReactorContext& context) noexcept
      -> const std::shared_ptr<ProgramContext>& {
    return context.program_context_;
  }
  [[nodiscard]] static auto get_program_context(const Context& context) noexcept
      -> const std::shared_ptr<ProgramContext>& {
    return std::visit([](auto& context) -> const auto& { return get_program_context(context); }, context);
  }

  [[nodiscard]] static auto get_source_location(const EnvironmentContext& context) noexcept
      -> detail::SourceLocationView {
    return context.source_location_;
  }
  [[nodiscard]] static auto get_source_location(const ReactorContext& context) noexcept -> detail::SourceLocationView {
    return context.source_location_;
  }
  [[nodiscard]] static auto get_source_location(const Context& context) noexcept -> detail::SourceLocationView {
    return std::visit([](auto& context) { return get_source_location(context); }, context);
  }
};

} // namespace xronos::sdk::detail

#endif // IMPL_XRONOS_SDK_DETAIL_CONTEXT_ACCESS_HH
