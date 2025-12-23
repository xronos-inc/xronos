// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_REACTION_HH
#define XRONOS_SDK_REACTION_HH

#include <any>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <variant>

#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/metric.hh"
#include "xronos/sdk/periodic_timer.hh"
#include "xronos/sdk/physical_event.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/shutdown.hh"
#include "xronos/sdk/startup.hh"
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
  ReactionContext(BaseReaction& reaction_instance)
      : reaction_instance_{reaction_instance} {}

  std::reference_wrapper<BaseReaction> reaction_instance_;
  [[nodiscard]] auto reaction_instance() const noexcept -> BaseReaction& { return reaction_instance_; }

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
  BaseReaction(const ReactionProperties& properties);

private:
  class TriggerImpl {
  protected:
    TriggerImpl(std::uint64_t trigger_uid, const ReactionContext& context);

    [[nodiscard]] auto get() const noexcept -> std::any;
    [[nodiscard]] auto is_present() const noexcept -> bool;

  private:
    std::uint64_t trigger_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;

    mutable const runtime::GettableTrigger* impl_{nullptr};
    [[nodiscard]] auto get_impl() const noexcept -> const runtime::GettableTrigger*;
  };

  class PortEffectImpl {
  protected:
    PortEffectImpl(std::uint64_t effect_uid, const ReactionContext& context);

    void set(const std::any& value) noexcept;
    [[nodiscard]] auto get() const noexcept -> std::any;
    [[nodiscard]] auto is_present() const noexcept -> bool;

  private:
    std::uint64_t effect_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;

    mutable runtime::SettableEffect* impl_{nullptr};
    [[nodiscard]] auto get_impl() noexcept -> runtime::SettableEffect*;
    [[nodiscard]] auto get_impl() const noexcept -> const runtime::SettableEffect*;
  };

  class ProgrammableTimerEffectImpl {
  protected:
    ProgrammableTimerEffectImpl(std::uint64_t effect_uid, const ReactionContext& context);

    void schedule(const std::any& value, Duration delay) noexcept;

  private:
    std::uint64_t effect_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;

    runtime::SchedulableEffect* impl_{nullptr};
    [[nodiscard]] auto get_impl() noexcept -> runtime::SchedulableEffect*;
  };

protected:
  /**
   * Get a context object for constructing reaction @ref
   * xronos::sdk::BaseReaction::Trigger "triggers" and @ref effects.
   *
   * @returns This reaction's context.
   */
  [[nodiscard]] auto context() noexcept -> auto { return ReactionContext{*this}; };

  /**
   * Declares a reaction trigger and provides read access to the triggering
   * EventSource.
   *
   * @tparam T The value type associated with events received on the triggering
   * event source.
   */
  template <class T> class Trigger : public TriggerImpl {
  public:
    /**
     * Constructor.
     *
     * Constructing a Trigger automatically registers the given event source as
     * a trigger of the reaction, causing the reaction handler to run whenever
     * the event source emits an event.
     *
     * @param trigger An event source that should trigger the reaction.
     * @param context Context of the reaction the trigger is declared for. Can
     * be obtained using context().
     */
    template <template <class> class Serializer>
    Trigger(const InputPort<T, Serializer>& trigger, const ReactionContext& context)
        : TriggerImpl{trigger.uid(), context} {}

    /** @overload */
    template <template <class> class Serializer>
    Trigger(const OutputPort<T, Serializer>& trigger, const ReactionContext& context)
        : TriggerImpl{trigger.uid(), context} {}

    /** @overload */
    Trigger(const PhysicalEvent<T>& trigger, const ReactionContext& context)
        : TriggerImpl{trigger.uid(), context} {}

    /** @overload */
    Trigger(const ProgrammableTimer<T>& trigger, const ReactionContext& context)
        : TriggerImpl{trigger.uid(), context} {}

    /** @overload */
    Trigger(const PeriodicTimer& trigger, const ReactionContext& context)
      requires(std::is_same_v<T, void>)
        : TriggerImpl{trigger.uid(), context} {}

    /** @overload */
    Trigger(const Startup& trigger, const ReactionContext& context)
      requires(std::is_same_v<T, void>)
        : TriggerImpl{trigger.uid(), context} {}

    /** @overload */
    Trigger(const Shutdown& trigger, const ReactionContext& context)
      requires(std::is_same_v<T, void>)
        : TriggerImpl{trigger.uid(), context} {}

    /**
     * Get the value of a currently present event.
     *
     * @returns A pointer to the value of the current event, or `nullptr` if there
     * is no current event (is_present() is false).
     */
    [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T>
      requires(!std::is_same_v<T, void>)
    {
      if (!is_present()) {
        return ImmutableValuePtr<T>{nullptr};
      }
      return std::any_cast<ImmutableValuePtr<T>>(TriggerImpl::get());
    }

    /**
     * Check if an event is present at the current timestamp.
     *
     * @returns `true` if an event is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return TriggerImpl::is_present(); }
  };

  /**
   * Allows a reaction to write data to a given Port.
   *
   * @tparam T The value type associated with the port.
   * @ingroup effects
   */
  template <class T> class PortEffect : public PortEffectImpl {
  public:
    /**
     * Constructor.
     *
     * @param port The Port for which the reaction should have write access.
     * @param context The context of the reaction the effect is declared for.
     * Can be obtained using context().
     */
    template <template <class> class Serializer>
    PortEffect(InputPort<T, Serializer>& port, const ReactionContext& context)
        : PortEffectImpl{port.uid(), context} {}

    /** @overload */
    template <template <class> class Serializer>
    PortEffect(OutputPort<T, Serializer>& port, const ReactionContext& context)
        : PortEffectImpl{port.uid(), context} {}

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
      PortEffectImpl::set(value_ptr);
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
      PortEffectImpl::set(std::monostate{});
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
      if (!is_present()) {
        return ImmutableValuePtr<T>{nullptr};
      }
      return std::any_cast<ImmutableValuePtr<T>>(PortEffectImpl::get());
    }

    /**
     * Check if an event is present at the current timestamp.
     *
     * @returns `true` if an event is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return PortEffectImpl::is_present(); }
  };

  /**
   * Allows a reaction to schedule future events using a ProgrammableTimer.
   *
   * @tparam T The value type associated with the programmable timer.
   * @ingroup effects
   */
  template <class T> class ProgrammableTimerEffect : public ProgrammableTimerEffectImpl {
  public:
    /**
     * Constructor.
     *
     * @param timer The ProgrammableTimer for which the reaction should have
     * write access.
     * @param context The context of the reaction the effect is declared for.
     * Can be obtained using context().
     */
    ProgrammableTimerEffect(ProgrammableTimer<T>& timer, const ReactionContext& context)
        : ProgrammableTimerEffectImpl{timer.uid(), context} {}

    /**
     * Schedule a future event.
     *
     * @param value_ptr The value to be associated with the future event occurrence.
     * @param delay The time to wait until the new event is processed.
     */
    void schedule(const ImmutableValuePtr<T>& value_ptr, Duration delay = Duration::zero())
      requires(!std::is_same_v<T, void>)
    {
      ProgrammableTimerEffectImpl::schedule(value_ptr, delay);
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
      ProgrammableTimerEffectImpl::schedule(std::monostate{}, delay);
    }

    // Disambiguate schedule(0) by explicitly deleting schedule(nullptr_t)
    template <typename V>
    void schedule(V, Duration delay = Duration::zero())
      requires(std::is_same_v<V, std::nullptr_t>)
    = delete;
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
    MetricEffect(Metric& metric, [[maybe_unused]] const ReactionContext& context)
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

  /**
   * Allows a reaction to terminate the program.
   */
  class ShutdownEffect {
  public:
    /**
     * Constructor.
     *
     * @param shutdown The shutdown event source used to trigger termination of the program.
     * @param context The context of the reaction the effect is declared for.
     * Can be obtained using context().
     */
    ShutdownEffect(Shutdown& shutdown, const ReactionContext& context);

    /**
     * Terminate the currently running reactor program.
     *
     * Terminates a running program at the next convenience. After completing all
     * currently active reactions, this triggers the Shutdown event sources. Once
     * all reactions triggered by Shutdown are processed, the program terminates.
     */
    void trigger_shutdown() noexcept;

  private:
    std::uint64_t effect_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;
    runtime::ShutdownEffect* impl_{nullptr};
    [[nodiscard]] auto get_impl() noexcept -> runtime::ShutdownEffect*;
  };

private:
  /**
   * The reaction handler.
   *
   * This method is invoked automatically in response to triggering events. User
   * code must override this method to define a reaction's behavior.
   */
  virtual void handler() = 0;

  using Element::core_element;
  using Element::program_context;
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
  Reaction(const ReactionProperties& properties)
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
