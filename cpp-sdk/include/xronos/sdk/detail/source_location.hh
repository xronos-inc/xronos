// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_DETAIL_SOURCE_LOCATION_HH
#define XRONOS_SDK_DETAIL_SOURCE_LOCATION_HH

#include <cstdint>
#include <source_location>
#include <string>

namespace xronos::sdk::detail {

struct SourceLocationView {
  std::string_view file;
  std::string_view function;
  std::uint32_t start_line{0};
  std::uint32_t end_line{0};
  std::uint32_t start_column{0};
  std::uint32_t end_column{0};

  [[nodiscard]] static auto from_std(std::source_location location) -> SourceLocationView {
    return SourceLocationView{location.file_name(), location.function_name(), location.line(), location.line()};
  }
};

struct SourceLocation {
  std::string file;
  std::string function;
  std::uint32_t start_line{0};
  std::uint32_t end_line{0};
  std::uint32_t start_column{0};
  std::uint32_t end_column{0};

  SourceLocation() = default;
  explicit SourceLocation(SourceLocationView location)
      : file{location.file}
      , function{location.function}
      , start_line{location.start_line}
      , end_line{location.end_line}
      , start_column{location.start_column}
      , end_column{location.end_column} {}
};

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_DETAIL_SOURCE_LOCATION_HH
