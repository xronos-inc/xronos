// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_REACTION_HH
#define XRONOS_SDK_REACTION_HH

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>

#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/metric.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/time.hh"
#include "xronos/sdk/value_ptr.hh"

/**
 * @defgroup effects effects Reaction effect classes.
 */

namespace xronos::sdk {
/**
 * Opaque data type used for constructing reaction @ref
 * xronos::sdk::BaseReaction::Trigger "triggers" and @ref effects.
 *
 * Use BaseReaction::context() to obtain an instance of this class.
 */
class ReactionContext {
private:
  constexpr ReactionContext(runtime::Reaction& reaction_instance)
      : reaction_instance_{reaction_instance} {}

  std::reference_wrapper<runtime::Reaction> reaction_instance_;
  constexpr auto reaction_instance() noexcept -> runtime::Reaction& { return reaction_instance_; }

  friend BaseReaction;
};

/**
 * Base class for implementing reactions.
 *
 * In the Xronos SDK, reactions define the behavior of a reactor. Reactions have
 * one or more @ref Trigger "triggers" and may have @ref effects. The reaction's
 * behavior is defined by overriding the handler() method, which is invoked
 * automatically for any event received on the triggers.
 *
 * Typically, user reactions should not inherit from BaseReaction directly and
 * use Reaction instead as it provides additional tools for accessing other
 * reactor elements and reactor state.
 *
 * Note that reaction classes may not be instantiated directly. Use the
 * Reactor::add_reaction() factory method instead.
 *
 * @see Reaction
 * @see Reactor::add_reaction()
 */
class BaseReaction : public Element {
public:
  /**
   * Constructor.
   *
   * Since ReactionProperties has no public constructor, this constructor cannot
   * be invoked directly. Use the Reactor::add_reaction() factory method
   * instead.
   */
  BaseReaction(ReactionProperties properties);

protected:
  /**
   * Get a context object for constructing reaction @ref
   * xronos::sdk::BaseReaction::Trigger "triggers" and @ref effects.
   *
   * @returns This reaction's context.
   */
  [[nodiscard]] auto context() noexcept -> ReactionContext;

  /**
   * Declares a reaction trigger and provides read access to the triggering
   * EventSource.
   *
   * @tparam T The value type associated with events received on the triggering
   * event source.
   */
  template <class T> class Trigger {
  public:
    /**
     * Constructor.
     *
     * Constructing a Trigger automatically registers the specified event source
     * with the reaction, causing the reaction handler to run whenever the event
     * source emits an event.
     *
     * @param trigger An EventSource source that should trigger the reaction.
     * @param context Context of the reaction the trigger is declared for. Can
     * be obtained using context().
     */
    Trigger(const EventSource<T>& trigger, ReactionContext context)
        : trigger_{trigger} {
      trigger.register_as_trigger_of(context.reaction_instance());
    }

    /**
     * Get the value of a currently present event.
     *
     * @returns A pointer to the value of the current event, or `nullptr` if there
     * is no current event (is_present() is false).
     */
    [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T>
      requires(!std::is_same_v<T, void>)
    {
      return trigger_.get().get();
    }

    /**
     * Check if an event is present at the current timestamp.
     *
     * @returns `true` if an event is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return trigger_.get().is_present(); }

  private:
    std::reference_wrapper<const EventSource<T>> trigger_;
  };

  /**
   * Allows a reaction to write data to a given Port.
   *
   * @tparam T The value type associated with the port.
   * @ingroup effects
   */
  template <class T> class PortEffect {
  public:
    /**
     * Constructor.
     *
     * @param port The Port for which the reaction should have write access.
     * @param context The context of the reaction the effect is declared for.
     * Can be obtained using context().
     */
    PortEffect(Port<T>& port, ReactionContext context)
        : port_{port} {
      detail::runtime_port::register_as_effect_of(port, context.reaction_instance());
    }

    /**
     * Write a value to the port sending a message to connected ports.
     *
     * May be called multiple times, but at most one value is sent to connected
     * ports. When called repeatedly at a given timestamp, the previous value is
     * overwritten.
     *
     * @param value_ptr A pointer to the value to be written to the referenced
     * port.
     */
    void set(const ImmutableValuePtr<T>& value_ptr)
      requires(!std::is_same_v<T, void>)
    {
      port_.get().set(value_ptr);
    }
    /**
     * @overload
     */
    void set(MutableValuePtr<T>&& value_ptr)
      requires(!std::is_same_v<T, void>)
    {
      set(ImmutableValuePtr<T>{std::move(value_ptr)});
    }
    /**
     * @overload
     *
     * @details Copy constructs the value using the given lvalue reference.
     */
    template <class U>
    void set(const U& value)
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      set(make_immutable_value<T>(value));
    }
    /**
     * @overload
     *
     * @details Move constructs the value using the given rvalue reference.
     */
    template <class U>
    void set(U&& value)
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      set(make_immutable_value<T>(std::forward<U>(value)));
    }

    /**
     * @overload
     *
     * @details Set the port without sending a value. This is only available if
     * `T` is `void`.
     */
    void set()
      requires(std::is_same_v<T, void>)
    {
      port_.get().set();
    }

    // Disambiguate set(0) by explicitly deleting set(nullptr_t)
    template <typename V>
    void set(V)
      requires(std::is_same_v<V, std::nullptr_t>)
    = delete;

    /**
     * Get a previously set value.
     *
     * @returns A pointer to the current value of the port
     * or `nullptr` if no value was set at the current timestamp.
     */
    [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T>
      requires(!std::is_same_v<T, void>)
    {
      return port_.get().get();
    }

    /**
     * Check if an event is present at the current timestamp.
     *
     * @returns `true` if an event is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return port_.get().is_present(); }

  private:
    std::reference_wrapper<Port<T>> port_;
  };

  /**
   * Allows a reaction to schedule future events using a ProgrammableTimer.
   *
   * @tparam T The value type associated with the programmable timer.
   * @ingroup effects
   */
  template <class T> class ProgrammableTimerEffect {
  public:
    /**
     * Constructor.
     *
     * @param timer The ProgrammableTimer for which the reaction should have
     * write access.
     * @param context The context of the reaction the effect is declared for.
     * Can be obtained using context().
     */
    ProgrammableTimerEffect(ProgrammableTimer<T>& timer, ReactionContext context)
        : event_{timer} {
      detail::runtime_programmable_timer::register_as_effect_of(timer, context.reaction_instance());
    }

    /**
     * Schedule a future event.
     *
     * @param value_ptr The value to be associated with the future event occurrence.
     * @param delay The time to wait until the new event is processed.
     */
    void schedule(const ImmutableValuePtr<T>& value_ptr, Duration delay = Duration::zero())
      requires(!std::is_same_v<T, void>)
    {
      event_.get().schedule(value_ptr, delay);
    }
    /**
     * @overload
     */
    void schedule(MutableValuePtr<T>&& value_ptr, Duration delay = Duration::zero())
      requires(!std::is_same_v<T, void>)
    {
      schedule(ImmutableValuePtr<T>{std::move(value_ptr)}, delay);
    }
    /**
     * @overload
     *
     * @details Copy constructs the value using the given lvalue reference.
     */
    template <class U>
    void schedule(const U& value, Duration delay = Duration::zero())
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      schedule(make_immutable_value<T>(value), delay);
    }
    /**
     * @overload
     *
     * @details Move constructs the value using the given rvalue reference.
     */
    template <class U>
    void schedule(U&& value, Duration delay = Duration::zero())
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      schedule(make_immutable_value<T>(std::forward<U>(value)), delay);
    }

    /**
     * @overload
     *
     * @details Schedule an event without an associated value. This is only
     * available if `T` is `void`.
     */
    void schedule(Duration delay = Duration::zero())
      requires(std::is_same_v<T, void>)
    {
      event_.get().schedule(delay);
    }

    // Disambiguate schedule(0) by explicitly deleting schedule(nullptr_t)
    template <typename V>
    void schedule(V, Duration delay = Duration::zero())
      requires(std::is_same_v<V, std::nullptr_t>)
    = delete;

  private:
    std::reference_wrapper<ProgrammableTimer<T>> event_;
  };

  /**
   * Allows a reaction to record telemetry data using a given Metric.
   *
   * @ingroup effects
   */
  class MetricEffect {
  public:
    /**
     * Constructor.
     *
     * @param metric The Metric for which the reaction should be able to record data.
     * @param context The context of the reaction the effect is declared for.
     * Can be obtained using context().
     */
    MetricEffect(Metric& metric, [[maybe_unused]] ReactionContext context)
        : metric_{metric} {}

    /**
     * Record a value at the current timestamp.
     *
     * @param value The value to record.
     */
    void record(double value) noexcept { metric_.get().record(value); }

    /**
     * @overload
     */
    void record(std::int64_t value) noexcept { metric_.get().record(value); }

  private:
    std::reference_wrapper<Metric> metric_;
  };

private:
  /**
   * The reaction handler.
   *
   * This method is invoked automatically in response to triggering events. User
   * code must override this method to define a reaction's behavior.
   */
  virtual void handler() = 0;
};

/**
 * Reaction base class with reactor access.
 *
 * In addition to BaseReaction, this class provides fully-typed access to the
 * owning reactor and all its state and elements.
 *
 * @tparam R Type of the owning reactor. This must be a subclass of Reactor.
 */
template <class R> class Reaction : public BaseReaction {
public:
  /**
   * @copydoc BaseReaction::BaseReaction
   */
  Reaction(ReactionProperties properties)
      : BaseReaction(properties)
      , self_(dynamic_cast<R&>(properties.container())) {}

protected:
  /**
   * Get a reference to the owning reactor.
   *
   * The reactor reference can be used for referencing other elements of the
   * reactor when declaring @ref xronos::sdk::BaseReaction::Trigger "triggers"
   * and @ref effects. It can also be used in the reaction @ref handler for
   * accessing the reactor's state.
   *
   * @returns A reference to the owning reactor.
   */
  [[nodiscard]] auto self() noexcept -> R& { return self_; }

private:
  std::reference_wrapper<R> self_;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_REACTION_HH
