// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2023 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_CONNECTION_PROPERTIES_HH
#define XRONOS_RUNTIME_CONNECTION_PROPERTIES_HH

#include "fwd.hh"
#include "logical_time.hh"
#include <cstdint>

namespace xronos::runtime {

enum class ConnectionType : std::uint8_t {
  Normal,
  Delayed,
  Enclaved,
  Physical,
  DelayedEnclaved,
  PhysicalEnclaved,
  Plugin,
  Invalid
};
struct ConnectionProperties {
  ConnectionType type_ = ConnectionType::Normal;
  Duration delay_{0};
  Environment* enclave_{nullptr};

  auto operator<(const ConnectionProperties& elem2) const noexcept -> bool {
    return (this->type_ < elem2.type_) || (this->type_ == elem2.type_ && this->delay_ < elem2.delay_);
  }

  auto operator==(const ConnectionProperties& elem2) const noexcept -> bool {
    return this->type_ == elem2.type_ && this->delay_ == elem2.delay_;
  }
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_CONNECTION_PROPERTIES_HH
