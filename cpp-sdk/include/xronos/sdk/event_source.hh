// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `EventSource` class.
 */

#ifndef XRONOS_SDK_EVENT_SOURCE_HH
#define XRONOS_SDK_EVENT_SOURCE_HH

#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

/**
 * @brief Base class for reactor elements that can be used as sources or
 * triggers of reactions.
 *
 * @details This usually should not be subclassed or instantiated by application
 * code. Subclasses provided by the SDK should be instantiated instead.
 * @tparam T The type of values conveyed by the event source.
 */
template <class T> class EventSource {
public:
  /**
   * @brief Correct deletion of an instance of a derived class is permitted.
   */
  virtual ~EventSource() = default;

private:
  [[nodiscard]] virtual auto get() const noexcept -> const ImmutableValuePtr<T>& = 0;
  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;

  virtual void register_as_trigger_of(runtime::Reaction& reaction) const noexcept = 0;
  virtual void register_as_source_of(runtime::Reaction& reaction) const noexcept = 0;

  friend BaseReaction;
};

/**
 * @brief Base class for reactor elements that can be used as sources or
 * triggers of reactions.
 *
 * @details This usually should not be subclassed or instantiated by application
 * code. Subclasses provided by the SDK should be instantiated instead.
 * @details This specialization is used for event sources that do not convey any
 * values.
 */
template <> class EventSource<void> {
public:
  /**
   * @brief Correct deletion of an instance of a derived class is permitted.
   */
  virtual ~EventSource() = default;

private:
  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;

  virtual void register_as_trigger_of(runtime::Reaction& reaction) const noexcept = 0;
  virtual void register_as_source_of(runtime::Reaction& reaction) const noexcept = 0;

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_EVENT_SOURCE_HH
