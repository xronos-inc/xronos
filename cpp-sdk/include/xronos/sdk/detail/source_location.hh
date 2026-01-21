// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_DETAIL_SOURCE_LOCATION_HH
#define XRONOS_SDK_DETAIL_SOURCE_LOCATION_HH

#include <cstdint>
#include <source_location>
#include <string_view>

namespace xronos::sdk::detail {

struct SourceLocationView {
  std::string_view file;
  std::string_view function;
  std::uint32_t start_line{0};
  std::uint32_t end_line{0};
  std::uint32_t start_column{0};
  std::uint32_t end_column{0};

  [[nodiscard]] static auto from_std(std::source_location location) -> SourceLocationView {
    return {.file = location.file_name(),
            .function = location.function_name(),
            .start_line = location.line(),
            .end_line = location.line()};
  }
};

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_DETAIL_SOURCE_LOCATION_HH
