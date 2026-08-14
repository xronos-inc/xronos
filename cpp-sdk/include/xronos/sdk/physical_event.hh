// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PHYSICAL_EVENT_HH
#define XRONOS_SDK_PHYSICAL_EVENT_HH

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/abi/value.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/value.hh"
#include "xronos/value/boxing.hh"

namespace xronos::sdk {

namespace detail {

inline auto register_physical_event(std::string_view name, const ReactorContext& context) -> std::uint64_t {
  return register_with_location(context, [&]() {
    return get_backend(context).register_physical_event(std::string{name}, ContextAccess::get_parent_uid(context));
  });
}

class UntypedPhysicalEvent {
public:
  UntypedPhysicalEvent(std::uint64_t uid, const ReactorContext& context)
      : uid_{uid}
      , program_context_{*ContextAccess::get_program_context(context)} {}

  void trigger(abi::AnyValue&& value) {
    if (auto* impl = get_impl(); impl != nullptr) {
      impl->trigger(std::move(value));
    }
  }

private:
  std::uint64_t uid_;
  std::reference_wrapper<const detail::ProgramContext> program_context_;

  // trigger() may be called from arbitrary threads (see abi::ExternalTrigger),
  // so the lazily resolved pointer is published atomically. Racing threads may
  // both perform the lookup; it returns the same pointer, so the double store
  // is benign.
  std::atomic<abi::ExternalTrigger*> impl_{nullptr};

  [[nodiscard]] auto get_impl() noexcept -> abi::ExternalTrigger* {
    auto* impl = impl_.load(std::memory_order_acquire);
    if (impl == nullptr) {
      // The lookup returns nullptr until the run is prepared; a non-null
      // result is fully visible to this thread (see abi::RuntimeBackend).
      impl = program_context_.get().runtime_backend().get_external_trigger(uid_);
      if (impl != nullptr) {
        impl_.store(impl, std::memory_order_release);
      }
    }

    return impl;
  }
};

} // namespace detail

/**
 * A reactor element for receiving events from external sources.
 *
 * Physical events may be used to trigger reactions from a context outside of
 * the scope of the reactor program. These could be external event handlers that
 * respond to sensor inputs.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger" allowing the
 * reaction handler to read the associated value.
 *
 * @tparam T The type of values carried by emitted events.
 */
template <class T> class PhysicalEvent final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the physical event.
   * @param context The containing reactor's context.
   */
  PhysicalEvent(std::string_view name, const ReactorContext& context)
      : Element{detail::register_physical_event(name, context), name, context}
      , impl_{uid(), context} {}

  /**
   * Emit a new event with an associated value and trigger reactions.
   *
   * The event will be assigned a timestamp equal to the current wall-clock
   * time.
   *
   * @param value The value to be associated with the emitted event.
   */
  template <class U>
  void trigger(const U& value)
    requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
  {
    impl_.trigger(xronos::value::make<T>(value));
  }
  /**
   * @overload
   * @details Move constructs the value using the given rvalue reference.
   */
  template <class U>
  void trigger(U&& value)
    requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
  {
    impl_.trigger(xronos::value::make<T>(std::forward<U>(value)));
  }
  /**
   * @overload
   * @details Emits the given value without copying it. The value must not
   * be empty (asserted in debug builds); nothing is emitted otherwise.
   */
  void trigger(const Value<T>& value)
    requires(!std::is_same_v<T, void>)
  {
    assert(value != nullptr && "an empty Value must not be sent");
    if (value != nullptr) {
      impl_.trigger(abi::AnyValue{detail::ValueAccess::unwrap(value)});
    }
  }
  /**
   * @overload
   * @details Emits the viewed value without copying it. The view must not
   * be absent (asserted in debug builds); nothing is emitted otherwise.
   */
  void trigger(const ValueView<T>& view)
    requires(!std::is_same_v<T, void>)
  {
    assert(view != nullptr && "an absent ValueView must not be sent");
    // Guard on the view's presence, not on the unwrapped pointer: a view
    // obtained from an absent trigger refers to an *empty* value, so the
    // pointer is non-null but nothing must be emitted.
    if (view != nullptr) {
      impl_.trigger(abi::AnyValue{*detail::ValueAccess::unwrap(view)});
    }
  }

  /**
   * @overload
   *
   * @details Emits an event without an associated value. This is only available if
   * `T` is `void`.
   */
  void trigger() noexcept
    requires(std::is_same_v<T, void>)
  {
    impl_.trigger(xronos::value::make<abi::Void>());
  }

  // Disambiguate trigger(0) by explicitly deleting trigger(nullptr_t)
  template <typename V>
  void trigger(V)
    requires(std::is_same_v<V, std::nullptr_t>)
  = delete;

private:
  detail::UntypedPhysicalEvent impl_;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PHYSICAL_EVENT_HH
