// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2023 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_CONNECTION_PROPERTIES_HH
#define XRONOS_RUNTIME_CONNECTION_PROPERTIES_HH

#include <cstdint>

#include "xronos/runtime/time.hh"

namespace xronos::runtime {

enum class ConnectionType : std::uint8_t {
  Normal,
  Delayed,
  Physical,
};
struct ConnectionProperties {
  ConnectionType type_ = ConnectionType::Normal;
  Duration delay_{0};

  auto operator<(const ConnectionProperties& elem2) const noexcept -> bool {
    return (this->type_ < elem2.type_) || (this->type_ == elem2.type_ && this->delay_ < elem2.delay_);
  }

  auto operator==(const ConnectionProperties& elem2) const noexcept -> bool {
    return this->type_ == elem2.type_ && this->delay_ == elem2.delay_;
  }
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_CONNECTION_PROPERTIES_HH
