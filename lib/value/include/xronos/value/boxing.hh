// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_VALUE_BOXING_HH
#define XRONOS_VALUE_BOXING_HH

#include <cstring>
#include <memory>
#include <new>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include "xronos/abi/value.hh"

// The boxing machinery behind `abi::AnyValue` (see xronos/abi/value.hh for
// the frozen contract): the per-payload-type managers, the storage strategy,
// and the typed API -- `make`, `holds`, `get_if`, `from_shared_ptr`,
// `as_shared_ptr`.
//
// Unlike the contract, everything in this component is a private, evolvable
// implementation detail of the side that boxes a value. Code that only
// stores, copies, moves, or destroys values -- ports, event queues,
// connections -- must depend on the abi contract only; without this header,
// payloads cannot be created or inspected, which enforces by construction
// that such code never interprets payloads.
//
// Evolution discipline: a change to the storage protocol implemented here
// must also change the manager symbol identity (an inline namespace bump on
// `detail`), so that processes mixing SDK versions never resolve one
// version's boxing code against another version's merged manager instance.

// This is a type-erasure layer: interpreting the storage union and treating
// its byte buffer as object storage is its whole purpose, in one audited
// place. The corresponding guideline checks are therefore suppressed for
// this file.
// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access, cppcoreguidelines-pro-bounds-array-to-pointer-decay)

namespace xronos::value {

namespace detail {

template <class T>
inline constexpr bool uses_inline_value_storage =
    std::is_trivially_copyable_v<T> && sizeof(T) <= abi::AnyValue::inline_capacity &&
    alignof(T) <= alignof(abi::ValueStorage);

// Shared payloads are managed by a `std::shared_ptr<const T>` that lives in
// the value's inline storage (16 bytes, within the 56-byte buffer). The
// shared_ptr is a private storage detail of the boxing side: it never
// crosses a library boundary itself, and every operation that touches it
// dispatches through the manager stamped into the value, so its internals
// never straddle two compilations. Storing a shared_ptr (rather than a
// bespoke control block) lets values share ownership with external
// `std::shared_ptr` holders in both directions without copying the payload
// (see `from_shared_ptr` and `as_shared_ptr`).
template <class T> using SharedPayload = std::shared_ptr<const T>;

template <class T> auto shared_payload(abi::ValueStorage& storage) noexcept -> SharedPayload<T>& {
  static_assert(sizeof(SharedPayload<T>) <= sizeof(storage.buf) &&
                alignof(SharedPayload<T>) <= alignof(abi::ValueStorage));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return *std::launder(reinterpret_cast<SharedPayload<T>*>(storage.buf));
}
template <class T> auto shared_payload(const abi::ValueStorage& storage) noexcept -> const SharedPayload<T>& {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return *std::launder(reinterpret_cast<const SharedPayload<T>*>(storage.buf));
}

// Deriving from `abi::ValueManager` grants access to the protected storage
// accessors -- the contract's one sanctioned door into `AnyValue`'s
// internals. The static boxing helpers live here (rather than on the free
// functions) for the same reason.
template <class T> class TypedValueManager final : public abi::ValueManager {
public:
  void copy(abi::AnyValue& dst, const abi::AnyValue& src) const final {
    if constexpr (uses_inline_value_storage<T>) {
      // Trivially copyable payloads are implicit-lifetime types; memcpy
      // creates the copy.
      std::memcpy(storage(dst).buf, storage(src).buf, sizeof(T));
    } else {
      ::new (static_cast<void*>(storage(dst).buf)) SharedPayload<T>(shared_payload<T>(storage(src)));
    }
    set_manager(dst, this);
  }

  void move(abi::AnyValue& dst, abi::AnyValue& src) const noexcept final {
    if constexpr (uses_inline_value_storage<T>) {
      std::memcpy(storage(dst).buf, storage(src).buf, sizeof(T));
    } else {
      auto& source = shared_payload<T>(storage(src));
      ::new (static_cast<void*>(storage(dst).buf)) SharedPayload<T>(std::move(source));
      std::destroy_at(&source);
    }
    set_manager(dst, this);
    set_manager(src, nullptr);
  }

  void destroy(abi::AnyValue& value) const noexcept final {
    if constexpr (!uses_inline_value_storage<T>) {
      std::destroy_at(&shared_payload<T>(storage(value)));
    }
    // Inline payloads are trivially destructible; nothing to do.
  }

  [[nodiscard]] auto payload(const abi::AnyValue& value) const noexcept -> const void* final {
    if constexpr (uses_inline_value_storage<T>) {
      return static_cast<const void*>(storage(value).buf);
    } else {
      return shared_payload<T>(storage(value)).get();
    }
  }

  [[nodiscard]] auto type_name() const noexcept -> const char* final { return typeid(T).name(); }

  // Boxing helpers for the free functions below; defined after
  // `typed_value_manager` so they can reference the merged instance.
  template <class... Args> [[nodiscard]] static auto box(Args&&... args) -> abi::AnyValue;
  [[nodiscard]] static auto box_shared(SharedPayload<T> pointer) -> abi::AnyValue;
  [[nodiscard]] static auto shared_of(const abi::AnyValue& value) -> SharedPayload<T>;
};

// One immortal instance per payload type. As an inline variable with default
// visibility this is a weak symbol, so the dynamic linker merges the
// instances of all objects that box the same type, making pointer identity a
// valid fast-path type check.
template <class T> inline const TypedValueManager<T> typed_value_manager{};

template <class T> template <class... Args> auto TypedValueManager<T>::box(Args&&... args) -> abi::AnyValue {
  abi::AnyValue result;
  if constexpr (uses_inline_value_storage<T>) {
    ::new (static_cast<void*>(storage(result).buf)) T(std::forward<Args>(args)...);
  } else {
    ::new (static_cast<void*>(storage(result).buf)) SharedPayload<T>(std::make_shared<T>(std::forward<Args>(args)...));
  }
  set_manager(result, &typed_value_manager<T>);
  return result;
}

template <class T> auto TypedValueManager<T>::box_shared(SharedPayload<T> pointer) -> abi::AnyValue {
  abi::AnyValue result;
  ::new (static_cast<void*>(storage(result).buf)) SharedPayload<T>(std::move(pointer));
  set_manager(result, &typed_value_manager<T>);
  return result;
}

template <class T> auto TypedValueManager<T>::shared_of(const abi::AnyValue& value) -> SharedPayload<T> {
  return shared_payload<T>(storage(value));
}

} // namespace detail

// True if the payload of `value` is of type `T`. Compares manager identity
// first and falls back to comparing mangled type names, mirroring how
// libstdc++ compares `std::type_info` across dynamically loaded objects.
template <class T> [[nodiscard]] auto holds(const abi::AnyValue& value) noexcept -> bool {
  static_assert(std::is_same_v<T, std::remove_cvref_t<T>>, "query decayed value types only");
  const auto* manager = value.manager();
  if (manager == nullptr) {
    return false;
  }
  if (manager == &detail::typed_value_manager<T>) {
    return true;
  }
  return std::strcmp(manager->type_name(), typeid(T).name()) == 0;
}

// Typed payload access; nullptr if `value` is empty or the payload is not a
// `T`. The pointer is valid as long as the value (or, for shared payloads,
// any copy of it) is alive and unmodified.
template <class T> [[nodiscard]] auto get_if(const abi::AnyValue& value) noexcept -> const T* {
  return holds<T>(value) ? static_cast<const T*>(value.payload()) : nullptr;
}

// Boxes a payload of type `T` constructed from `args`.
template <class T, class... Args> [[nodiscard]] auto make(Args&&... args) -> abi::AnyValue {
  static_assert(std::is_same_v<T, std::remove_cvref_t<T>>, "box decayed value types only");
  return detail::TypedValueManager<T>::box(std::forward<Args>(args)...);
}

// Boxes a payload that is already managed by a `std::shared_ptr`, sharing
// ownership with the given pointer instead of copying the payload. The
// payload must not be modified through any other reference for as long as
// any value shares it -- the same contract as sharing a
// `std::shared_ptr<const T>`. A `std::unique_ptr<T>` converts implicitly,
// handing over exclusive ownership with no aliasing at all.
//
// Payload types that qualify for inline storage are copied out of the given
// pointer instead of shared (which is cheaper than sharing for such types).
// A null pointer yields the empty value.
template <class T> [[nodiscard]] auto from_shared_ptr(std::shared_ptr<const T> pointer) -> abi::AnyValue {
  static_assert(std::is_same_v<T, std::remove_cvref_t<T>>, "box decayed value types only");
  if (pointer == nullptr) {
    return abi::AnyValue{};
  }
  if constexpr (detail::uses_inline_value_storage<T>) {
    return detail::TypedValueManager<T>::box(*pointer);
  } else {
    return detail::TypedValueManager<T>::box_shared(std::move(pointer));
  }
}

// Retrieves the payload as a `std::shared_ptr`. For payloads in shared
// storage the returned pointer shares ownership with the value and pointer
// identity is preserved; payloads in inline storage are copied into a fresh
// allocation on every call. Returns null if the value is empty or does not
// hold a `T`.
template <class T> [[nodiscard]] auto as_shared_ptr(const abi::AnyValue& value) -> std::shared_ptr<const T> {
  if (!holds<T>(value)) {
    return nullptr;
  }
  if constexpr (detail::uses_inline_value_storage<T>) {
    return std::make_shared<T>(*static_cast<const T*>(value.payload()));
  } else {
    return detail::TypedValueManager<T>::shared_of(value);
  }
}

} // namespace xronos::value

// NOLINTEND(cppcoreguidelines-pro-type-union-access, cppcoreguidelines-pro-bounds-array-to-pointer-decay)

#endif // XRONOS_VALUE_BOXING_HH
