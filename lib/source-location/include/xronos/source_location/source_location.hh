// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SOURCE_LOCATION_SOURCE_LOCATION_HH
#define XRONOS_SOURCE_LOCATION_SOURCE_LOCATION_HH

#include <cstdint>
#include <ranges>
#include <string>
#include <unordered_map>
#include <utility>

#include "xronos/util/assert.hh"

namespace xronos::source_location {

struct SourceLocation {
  std::string file;
  std::string function;
  std::uint32_t start_line{0};
  std::uint32_t end_line{0};
  std::uint32_t start_column{0};
  std::uint32_t end_column{0};
};

class SourceLocationRegistry {
public:
  void add_source_location(std::uint64_t uid, SourceLocation&& source_location) {
    auto res = source_locations_.try_emplace(uid, std::move(source_location));
    util::assert_(res.second);
  }

  [[nodiscard]] auto all_locations() const noexcept -> auto { return std::views::all(source_locations_); }

private:
  std::unordered_map<std::uint64_t, SourceLocation> source_locations_{};
};

} // namespace xronos::source_location

#endif // XRONOS_SOURCE_LOCATION_SOURCE_LOCATION_HH
