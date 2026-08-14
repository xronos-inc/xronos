// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_ABI_VALUE_HH
#define XRONOS_ABI_VALUE_HH

#include <cstddef>
#include <type_traits>
#include <utility>

// Included for its compile-time toolchain checks: every translation unit
// that touches the ABI -- including one that only boxes values -- must
// satisfy the contract.
#include "xronos/abi/contract.hh" // IWYU pragma: keep

// Type-erased value passing for reactor messages.
//
// `AnyValue` is the container for port, event, and physical-event payloads.
// Unlike `std::any`, its layout and its dispatch protocol are fully under
// our control: the struct layout is frozen (see the static_asserts below)
// and every copy/move/destroy operation dispatches through a per-type
// `ValueManager` vtable emitted by the code that boxed the value. Code that
// merely stores, copies, moves, or destroys an `AnyValue` never interprets
// the payload storage itself, so the storage strategy of the boxing side
// (inline vs. refcounted heap) remains a private, evolvable implementation
// detail behind the manager.
//
// This header is the CONTRACT only: the frozen layout and the manager
// interface. The boxing machinery -- the per-type managers, the storage
// strategy, and the typed API (`value::make`, `value::holds`,
// `value::get_if`, `value::from_shared_ptr`, `value::as_shared_ptr`) --
// lives in the separate xronos-value component (xronos/value/boxing.hh),
// keeping everything under xronos/abi frozen. Code that only stores,
// copies, moves, or destroys values depends on this header alone and is
// thereby structurally unable to create or inspect payloads.
//
// FROZEN: `AnyValue` is 64 bytes (8-byte manager pointer + 56-byte storage
// union) with 8-byte alignment -- one cache line. This layout must never
// change; new capabilities are added behind `ValueManager`, not by growing
// the struct. The size was chosen empirically: inline storage beats shared
// storage at every payload size that fits, and 64 is the largest total size
// whose cache-footprint cost is not measurable.

namespace xronos::abi::inline v1 {

class AnyValue;

// The boxed representation of `void` payloads: events of `void` type carry a
// `Void` value, so that a present event always holds a non-empty value and
// presence is equivalent to `AnyValue::has_value()`. (For the type-minded:
// this is a unit type -- it has exactly one value and carries no information
// beyond its existence.) The empty `AnyValue` uniformly means "no value" and
// is never sent through ports.
struct Void {};

// The storage of an `AnyValue`: either a pointer to heap storage or a small
// payload stored inline. Which of the two is active is known only to the
// `ValueManager` that created the value; code outside the manager must not
// interpret this union.
union ValueStorage {
  void* ptr;
  // Deliberately a raw array, not std::array: the layout of the frozen
  // 56-byte buffer is guaranteed by the language itself rather than by
  // library convention.
  // NOLINTNEXTLINE(*-avoid-c-arrays, *-magic-numbers) frozen layout, see above
  alignas(8) std::byte buf[56];
};

// Per-payload-type manager that implements copy/move/destroy and payload
// access for `AnyValue`. Exactly one (stateless, immortal) instance exists
// per payload type, emitted as a weak symbol by the translation unit that
// boxes values of that type; the manager pointer therefore doubles as a fast
// type-identity check (with `type_name` as the cross-DSO fallback).
//
// Contract for all methods: `dst` is empty (no payload, manager unset) on
// entry to `copy` and `move`; `move` leaves `src` empty; `destroy` releases
// the payload but leaves clearing the manager pointer to the caller. All
// methods may be called from arbitrary threads; `copy` may throw (allocation
// or payload copy), `move` and `destroy` must not.
class ValueManager {
public:
  virtual void copy(AnyValue& dst, const AnyValue& src) const = 0;
  virtual void move(AnyValue& dst, AnyValue& src) const noexcept = 0;
  virtual void destroy(AnyValue& value) const noexcept = 0;
  // Pointer to the managed payload object. Valid as long as `value` is alive
  // and unmodified.
  [[nodiscard]] virtual auto payload(const AnyValue& value) const noexcept -> const void* = 0;
  // The Itanium-mangled name of the payload type (`typeid(T).name()`),
  // identical for GCC and Clang. Used as the type-identity fallback when
  // comparing manager instances across independently loaded objects.
  [[nodiscard]] virtual auto type_name() const noexcept -> const char* = 0;

protected:
  // Managers are immortal statics; nothing destroys them through the
  // interface.
  ~ValueManager() = default;

  // Storage access for implementations (`AnyValue`'s members are private).
  static auto storage(AnyValue& value) noexcept -> ValueStorage&;
  static auto storage(const AnyValue& value) noexcept -> const ValueStorage&;
  static void set_manager(AnyValue& value, const ValueManager* manager) noexcept;
};

// Type-erased owned value. Empty by default; the typed API for boxing and
// inspecting payloads (`value::make<T>`, `value::get_if<T>`, ...) lives in
// the xronos-value component. The empty state uniformly means "no value":
// event presence is equivalent to the stored value being non-empty, and
// events of `void` type carry a boxed `Void` rather than the empty value.
class AnyValue {
public:
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-union-access)
  static constexpr std::size_t inline_capacity = sizeof(ValueStorage::buf);

  constexpr AnyValue() noexcept = default;

  AnyValue(const AnyValue& other) {
    if (other.manager_ != nullptr) {
      other.manager_->copy(*this, other);
    }
  }

  AnyValue(AnyValue&& other) noexcept {
    if (other.manager_ != nullptr) {
      other.manager_->move(*this, other);
    }
  }

  auto operator=(const AnyValue& other) -> AnyValue& {
    if (this != &other) {
      AnyValue copy{other};
      *this = std::move(copy);
    }
    return *this;
  }

  auto operator=(AnyValue&& other) noexcept -> AnyValue& {
    if (this != &other) {
      reset();
      if (other.manager_ != nullptr) {
        other.manager_->move(*this, other);
      }
    }
    return *this;
  }

  ~AnyValue() { reset(); }

  // Destroys the payload (or releases its shared reference), leaving the
  // value empty.
  void reset() noexcept {
    if (manager_ != nullptr) {
      manager_->destroy(*this);
      manager_ = nullptr;
    }
  }

  [[nodiscard]] auto has_value() const noexcept -> bool { return manager_ != nullptr; }

  [[nodiscard]] auto manager() const noexcept -> const ValueManager* { return manager_; }

  [[nodiscard]] auto payload() const noexcept -> const void* {
    return manager_ != nullptr ? manager_->payload(*this) : nullptr;
  }

private:
  friend class ValueManager;

  const ValueManager* manager_{nullptr};
  ValueStorage storage_{};
};

// The frozen layout. Changing any of these is an ABI break. The literals
// are the contract, deliberately not named constants.
// NOLINTBEGIN(*-magic-numbers)
static_assert(sizeof(AnyValue) == 64);
static_assert(alignof(AnyValue) == 8);
static_assert(std::is_standard_layout_v<AnyValue>);
// NOLINTEND(*-magic-numbers)

inline auto ValueManager::storage(AnyValue& value) noexcept -> ValueStorage& { return value.storage_; }
inline auto ValueManager::storage(const AnyValue& value) noexcept -> const ValueStorage& { return value.storage_; }
inline void ValueManager::set_manager(AnyValue& value, const ValueManager* manager) noexcept {
  value.manager_ = manager;
}

} // namespace xronos::abi::inline v1

#endif // XRONOS_ABI_VALUE_HH
