// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_EVENT_SOURCE_HH
#define XRONOS_SDK_EVENT_SOURCE_HH

#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

/**
 * Interface implemented by reactor elements that can be used as @ref
 * BaseReaction::Trigger "triggers" of reactions.
 *
 * @tparam T Value type associated with events emitted by this event source.
 */
template <class T> class EventSource {
public:
  virtual ~EventSource() = default;

private:
  [[nodiscard]] virtual auto get() const noexcept -> ImmutableValuePtr<T> = 0;
  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;

  virtual void register_as_trigger_of(runtime::Reaction& reaction) const noexcept = 0;

  friend BaseReaction;
};

/**
 * @copybrief EventSource
 *
 * This is a template specialization of EventSource for event sources that emit
 * events without an associated value.
 */
template <> class EventSource<void> {
public:
  virtual ~EventSource() = default;

private:
  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;

  virtual void register_as_trigger_of(runtime::Reaction& reaction) const noexcept = 0;

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_EVENT_SOURCE_HH
