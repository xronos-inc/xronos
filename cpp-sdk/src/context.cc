// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/context.hh"
#include "xronos/sdk/environment.hh"

namespace xronos::sdk::detail {

void store_source_location(ReactorContext context, std::uint64_t uid, std::string_view fqn) {
  store_source_location(context.environment_, uid, fqn, context.source_location_);
}

void store_source_location(EnvironmentContext context, std::uint64_t uid, std::string_view fqn) {
  store_source_location(context.environment_, uid, fqn, context.source_location_);
}

void store_source_location(Context context, std::uint64_t uid, std::string_view fqn) {
  std::visit([uid, fqn](auto ctx) { store_source_location(ctx, uid, fqn); }, context);
}

} // namespace xronos::sdk::detail
