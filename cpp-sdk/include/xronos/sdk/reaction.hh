// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_REACTION_HH
#define XRONOS_SDK_REACTION_HH

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/abi/value.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/element.hh"
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
#include "xronos/sdk/value.hh"
#include "xronos/value/boxing.hh"

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

namespace detail {

[[nodiscard]] inline auto get_reaction_time_access(const detail::ProgramContext& program_context,
                                                   std::uint64_t reactor_uid) noexcept -> const abi::TimeAccess* {
  // Null until the run is prepared (see abi::RuntimeBackend).
  return program_context.runtime_backend().get_time_access(reactor_uid);
}

} // namespace detail

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
  BaseReaction(const ReactionProperties& properties)
      : Element{register_reaction(properties, this), properties.name_, properties.context_}
      , parent_uid_{detail::ContextAccess::get_parent_uid(properties.context_)}
      , deadline_{properties.deadline_} {}

protected:
  /**
   * Get the deadline associated with the current handler invocation.
   *
   * If the reaction was created using Reactor::add_reaction_with_deadline() with
   * a deadline duration D, this returns the wall-clock instant by which the
   * handler must complete, equal to current_time() + D. It is therefore anchored
   * to current_time() and stays fixed for the duration of the handler. The
   * handler meets its deadline if it completes before the wall clock reaches
   * this instant.
   *
   * This should only be called from the reaction handler. If called outside of
   * program execution (e.g. during reaction declaration), this returns
   * std::nullopt.
   *
   * @returns The wall-clock instant by which the handler must complete (equal to
   * current_time() + D), or std::nullopt if no deadline is set or the program is
   * not executing.
   */
  [[nodiscard]] auto deadline() const noexcept -> std::optional<TimePoint>;

  /**
   * Get the remaining slack before the deadline.
   *
   * This is the remaining wall-clock duration before the deadline, computed as
   * deadline() minus the current wall-clock reading. Equivalently, for a
   * declared deadline duration D it is D - lag(): the lag and the slack always
   * sum to D, so as the lag grows during the handler the slack shrinks by the
   * same amount. The slack denotes how much further the lag may grow before the
   * deadline is violated. A negative value means the deadline has been missed:
   * the wall clock has passed the deadline.
   *
   * This should only be called from the reaction handler.
   *
   * @returns The remaining wall-clock duration before the deadline. If there is
   * no deadline, or if the program is not executing (e.g. during reaction
   * declaration), Duration::max() is returned.
   */
  [[nodiscard]] auto slack() const noexcept -> Duration {
    auto deadline_value = deadline();
    if (!deadline_value.has_value()) {
      return Duration::max();
    }
    return *deadline_value - std::chrono::system_clock::now();
  }

  /**
   * Check whether the currently executing handler is still within its deadline.
   *
   * This should only be called from the reaction handler.
   *
   * @returns true while the wall clock has not yet reached the deadline (the
   * slack is positive), and false once the deadline has been missed. If no
   * deadline was declared, this is always true.
   */
  [[nodiscard]] auto is_before_deadline() const noexcept -> bool { return slack() > Duration::zero(); }

  /**
   * Get the current time.
   *
   * This does not read wall-clock time. The Xronos runtime uses an internal
   * clock to control how a program advances, and this returns the current
   * reading of that clock. The internal clock does not advance while a reaction
   * handler executes, so this value does not change while the handler runs: any
   * two reads within the same handler return the same value.
   *
   * This is a reaction-scoped accessor and is only meaningful while the
   * reaction handler executes; values read outside a handler must not be
   * relied upon. If the program is not yet executing (for example, when called
   * during the reaction's construction or member initialization), this returns
   * a default value: the epoch (a default-constructed TimePoint).
   *
   * @returns The current time as provided by the internal clock.
   */
  [[nodiscard]] auto current_time() const noexcept -> TimePoint {
    const auto* time_access = detail::get_reaction_time_access(*program_context(), parent_uid_);
    if (time_access == nullptr) {
      return TimePoint{};
    }
    return time_access->get_timestamp();
  }

  /**
   * Get the current lag.
   *
   * The lag is the difference between wall-clock time and the current time,
   * computed as the current wall-clock reading minus current_time(). It relates
   * the internal clock to the advancing wall clock and therefore changes while
   * the handler runs: the current time does not advance, but the wall clock
   * does, so the lag measures how far the wall clock has run ahead of the
   * internal clock -- that is, how far the execution of reactions lags behind
   * the events it processes.
   *
   * This is a reaction-scoped accessor and is only meaningful while the
   * reaction handler executes; values read outside a handler must not be
   * relied upon. If the program is not yet executing (for example, when called
   * during the reaction's construction or member initialization), this returns
   * a default value: zero.
   *
   * @returns The current lag as a wall-clock duration.
   */
  [[nodiscard]] auto lag() const noexcept -> Duration {
    const auto* time_access = detail::get_reaction_time_access(*program_context(), parent_uid_);
    if (time_access == nullptr) {
      return Duration::zero();
    }
    return std::chrono::system_clock::now() - time_access->get_timestamp();
  }

  /**
   * Get how far the internal clock has advanced since the startup event.
   *
   * This is the difference between the current time and the time at which the
   * program started. It is measured on the internal clock and does not depend
   * on wall-clock time. Like current_time(), it does not change while a
   * reaction handler executes.
   *
   * This is a reaction-scoped accessor and is only meaningful while the
   * reaction handler executes; values read outside a handler must not be
   * relied upon. If the program is not yet executing (for example, when called
   * during the reaction's construction or member initialization), this returns
   * a default value: zero.
   *
   * @returns The difference between the current time given by current_time()
   * and the time at which the program started.
   */
  [[nodiscard]] auto elapsed_time() const noexcept -> Duration {
    const auto* time_access = detail::get_reaction_time_access(*program_context(), parent_uid_);
    if (time_access == nullptr) {
      return Duration::zero();
    }
    return time_access->get_timestamp() - time_access->get_start_timestamp();
  }

private:
  // Invokes the user's reaction body through the ABI's reaction-handler
  // interface. The reaction must outlive the handler, which the backend owns.
  class HandlerImpl final : public abi::ReactionHandler {
  public:
    explicit HandlerImpl(BaseReaction& reaction)
        : reaction_{reaction} {}

    void invoke() final { reaction_.get().handler(); }

  private:
    std::reference_wrapper<BaseReaction> reaction_;
  };

  class UntypedTrigger {
  protected:
    UntypedTrigger(std::uint64_t trigger_uid, const ReactionContext& context)
        : trigger_uid_{trigger_uid}
        , reaction_uid_{context.reaction_instance().uid()}
        , program_context_{*context.reaction_instance().program_context()} {
      context.reaction_instance().program_context()->backend().register_reaction_trigger(reaction_uid_, trigger_uid_);
    }

    [[nodiscard]] auto get() const noexcept -> const abi::AnyValue& {
      if (const auto* impl = get_impl(); impl != nullptr) {
        return impl->get();
      }
      static const abi::AnyValue empty{};
      return empty;
    }

    [[nodiscard]] auto is_present() const noexcept -> bool {
      if (const auto* impl = get_impl(); impl != nullptr) {
        return impl->get().has_value();
      }
      return false;
    }

  private:
    std::uint64_t trigger_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;

    mutable const abi::GettableTrigger* impl_{nullptr};
    [[nodiscard]] auto get_impl() const noexcept -> const abi::GettableTrigger* {
      if (impl_ == nullptr) {
        impl_ = program_context_.get().runtime_backend().get_trigger(reaction_uid_, trigger_uid_);
        assert(impl_ != nullptr);
      }
      return impl_;
    }
  };

  class UntypedPortEffect {
  protected:
    UntypedPortEffect(std::uint64_t effect_uid, const ReactionContext& context)
        : effect_uid_{effect_uid}
        , reaction_uid_{context.reaction_instance().uid()}
        , program_context_{*context.reaction_instance().program_context()} {
      context.reaction_instance().program_context()->backend().register_reaction_effect(reaction_uid_, effect_uid_);
    }

    void set(abi::AnyValue&& value) noexcept {
      if (auto* impl = get_impl(); impl != nullptr) {
        impl->set(std::move(value));
      }
    }

    [[nodiscard]] auto get() const noexcept -> const abi::AnyValue& {
      if (const auto* impl = get_impl(); impl != nullptr) {
        return impl->get();
      }
      static const abi::AnyValue empty{};
      return empty;
    }

    [[nodiscard]] auto is_present() const noexcept -> bool {
      if (const auto* impl = get_impl(); impl != nullptr) {
        return impl->get().has_value();
      }
      return false;
    }

  private:
    std::uint64_t effect_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;

    mutable abi::SettableEffect* impl_{nullptr};
    [[nodiscard]] auto get_impl() noexcept -> abi::SettableEffect* {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
      return const_cast<abi::SettableEffect*>(std::as_const(*this).get_impl());
    }
    [[nodiscard]] auto get_impl() const noexcept -> const abi::SettableEffect* {
      if (impl_ == nullptr) {
        impl_ = program_context_.get().runtime_backend().get_settable_effect(reaction_uid_, effect_uid_);
        assert(impl_ != nullptr);
      }
      return impl_;
    }
  };

  class UntypedProgrammableTimerEffect {
  protected:
    UntypedProgrammableTimerEffect(std::uint64_t effect_uid, const ReactionContext& context)
        : effect_uid_{effect_uid}
        , reaction_uid_{context.reaction_instance().uid()}
        , program_context_{*context.reaction_instance().program_context()} {
      context.reaction_instance().program_context()->backend().register_reaction_effect(reaction_uid_, effect_uid_);
    }

    void schedule(abi::AnyValue&& value, Duration delay) noexcept {
      if (auto* impl = get_impl(); impl != nullptr) {
        impl->schedule(std::move(value), delay);
      }
    }

  private:
    std::uint64_t effect_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;

    abi::SchedulableEffect* impl_{nullptr};
    [[nodiscard]] auto get_impl() noexcept -> abi::SchedulableEffect* {
      if (impl_ == nullptr) {
        impl_ = program_context_.get().runtime_backend().get_schedulable_effect(reaction_uid_, effect_uid_);
        assert(impl_ != nullptr);
      }
      return impl_;
    }
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
  template <class T> class Trigger : public UntypedTrigger {
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
        : UntypedTrigger{trigger.uid(), context} {}

    /** @overload */
    template <template <class> class Serializer>
    Trigger(const OutputPort<T, Serializer>& trigger, const ReactionContext& context)
        : UntypedTrigger{trigger.uid(), context} {}

    /** @overload */
    Trigger(const PhysicalEvent<T>& trigger, const ReactionContext& context)
        : UntypedTrigger{trigger.uid(), context} {}

    /** @overload */
    Trigger(const ProgrammableTimer<T>& trigger, const ReactionContext& context)
        : UntypedTrigger{trigger.uid(), context} {}

    /** @overload */
    Trigger(const PeriodicTimer& trigger, const ReactionContext& context)
      requires(std::is_same_v<T, void>)
        : UntypedTrigger{trigger.uid(), context} {}

    /** @overload */
    Trigger(const Startup& trigger, const ReactionContext& context)
      requires(std::is_same_v<T, void>)
        : UntypedTrigger{trigger.uid(), context} {}

    /** @overload */
    Trigger(const Shutdown& trigger, const ReactionContext& context)
      requires(std::is_same_v<T, void>)
        : UntypedTrigger{trigger.uid(), context} {}

    /**
     * Get a view of the current event's value.
     *
     * @returns A ValueView of the current event's value. The view is absent
     * (evaluates to `false`) if no event is present at the current time.
     * The view is only valid for the duration of the current reaction
     * handler; copy it into a Value to keep it alive beyond that.
     */
    [[nodiscard]] auto get() const noexcept -> ValueView<T>
      requires(!std::is_same_v<T, void>)
    {
      return detail::ValueAccess::view<T>(UntypedTrigger::get());
    }

    /**
     * Check if an event is present at the current timestamp.
     *
     * @returns `true` if an event is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return UntypedTrigger::is_present(); }
  };

  /**
   * Allows a reaction to write data to a given Port.
   *
   * @tparam T The value type associated with the port.
   * @ingroup effects
   */
  template <class T> class PortEffect : public UntypedPortEffect {
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
        : UntypedPortEffect{port.uid(), context} {}

    /** @overload */
    template <template <class> class Serializer>
    PortEffect(OutputPort<T, Serializer>& port, const ReactionContext& context)
        : UntypedPortEffect{port.uid(), context} {}

    /**
     * Write a value to the port, sending a message to connected ports.
     *
     * May be called multiple times, but at most one message is sent to
     * connected ports at any given time timestamp: when called repeatedly,
     *  the previously written value is replaced.
     *
     * @param value The value to be written to the referenced port. Copy
     * constructs the value from the given lvalue reference.
     */
    template <class U>
    void set(const U& value)
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      UntypedPortEffect::set(xronos::value::make<T>(value));
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
      UntypedPortEffect::set(xronos::value::make<T>(std::forward<U>(value)));
    }
    /**
     * @overload
     *
     * @details Sends the given value without copying it. The value must not
     * be empty (asserted in debug builds); nothing is sent otherwise.
     */
    void set(const Value<T>& value)
      requires(!std::is_same_v<T, void>)
    {
      assert(value != nullptr && "an empty Value must not be sent");
      if (value != nullptr) {
        UntypedPortEffect::set(abi::AnyValue{detail::ValueAccess::unwrap(value)});
      }
    }
    /**
     * @overload
     *
     * @details Sends the viewed value without copying it. Use this to relay
     * a value received on a trigger. The view must not be absent (asserted
     * in debug builds); nothing is sent otherwise.
     */
    void set(const ValueView<T>& view)
      requires(!std::is_same_v<T, void>)
    {
      assert(view != nullptr && "an absent ValueView must not be sent");
      // Guard on the view's presence, not on the unwrapped pointer: a view
      // obtained from an absent trigger refers to an *empty* value, so the
      // pointer is non-null but nothing must be sent.
      if (view != nullptr) {
        UntypedPortEffect::set(abi::AnyValue{*detail::ValueAccess::unwrap(view)});
      }
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
      UntypedPortEffect::set(xronos::value::make<abi::Void>());
    }

    // Disambiguate set(0) by explicitly deleting set(nullptr_t)
    template <typename V>
    void set(V)
      requires(std::is_same_v<V, std::nullptr_t>)
    = delete;

    /**
     * Get a view of a previously set value.
     *
     * @returns A ValueView of the port's current value. The view is absent
     * (evaluates to `false`) if no value was set at the current timestamp.
     * The view is only valid for the duration of the current reaction
     * handler; copy it into a Value to keep it alive beyond that.
     */
    [[nodiscard]] auto get() const noexcept -> ValueView<T>
      requires(!std::is_same_v<T, void>)
    {
      return detail::ValueAccess::view<T>(UntypedPortEffect::get());
    }

    /**
     * Check if an event is present at the current timestamp.
     *
     * @returns `true` if an event is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return UntypedPortEffect::is_present(); }
  };

  /**
   * Allows a reaction to schedule future events using a ProgrammableTimer.
   *
   * @tparam T The value type associated with the programmable timer.
   * @ingroup effects
   */
  template <class T> class ProgrammableTimerEffect : public UntypedProgrammableTimerEffect {
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
        : UntypedProgrammableTimerEffect{timer.uid(), context} {}

    /**
     * Schedule a future event.
     *
     * @param value The value to be associated with the future event
     * occurrence. Copy constructs the value from the given lvalue reference.
     * @param delay The time to wait until the new event is processed.
     */
    template <class U>
    void schedule(const U& value, Duration delay = Duration::zero())
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      UntypedProgrammableTimerEffect::schedule(xronos::value::make<T>(value), delay);
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
      UntypedProgrammableTimerEffect::schedule(xronos::value::make<T>(std::forward<U>(value)), delay);
    }
    /**
     * @overload
     *
     * @details Schedules the given value without copying it. The value must
     * not be empty (asserted in debug builds); nothing is scheduled
     * otherwise.
     */
    void schedule(const Value<T>& value, Duration delay = Duration::zero())
      requires(!std::is_same_v<T, void>)
    {
      assert(value != nullptr && "an empty Value must not be scheduled");
      if (value != nullptr) {
        UntypedProgrammableTimerEffect::schedule(abi::AnyValue{detail::ValueAccess::unwrap(value)}, delay);
      }
    }
    /**
     * @overload
     *
     * @details Schedules the viewed value without copying it. The view must
     * not be absent (asserted in debug builds); nothing is scheduled
     * otherwise.
     */
    void schedule(const ValueView<T>& view, Duration delay = Duration::zero())
      requires(!std::is_same_v<T, void>)
    {
      assert(view != nullptr && "an absent ValueView must not be scheduled");
      // Guard on the view's presence, not on the unwrapped pointer: a view
      // obtained from an absent trigger refers to an *empty* value, so the
      // pointer is non-null but nothing must be scheduled.
      if (view != nullptr) {
        UntypedProgrammableTimerEffect::schedule(abi::AnyValue{*detail::ValueAccess::unwrap(view)}, delay);
      }
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
      UntypedProgrammableTimerEffect::schedule(xronos::value::make<abi::Void>(), delay);
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
    ShutdownEffect(Shutdown& shutdown, const ReactionContext& context)
        : effect_uid_{shutdown.uid()}
        , reaction_uid_{context.reaction_instance().uid()}
        , program_context_{*context.reaction_instance().program_context()} {
      context.reaction_instance().program_context()->backend().register_reaction_effect(reaction_uid_, effect_uid_);
    }

    /**
     * Terminate the currently running reactor program.
     *
     * Terminates a running program at the next convenience. After completing all
     * currently active reactions, this triggers the Shutdown event sources. Once
     * all reactions triggered by Shutdown are processed, the program terminates.
     */
    void trigger_shutdown() noexcept {
      if (auto* impl = get_impl(); impl != nullptr) {
        impl->trigger_shutdown();
      }
    }

  private:
    std::uint64_t effect_uid_;
    std::uint64_t reaction_uid_;
    std::reference_wrapper<const detail::ProgramContext> program_context_;
    abi::ShutdownEffect* impl_{nullptr};
    [[nodiscard]] auto get_impl() noexcept -> abi::ShutdownEffect* {
      if (impl_ == nullptr) {
        impl_ = program_context_.get().runtime_backend().get_shutdown_effect(reaction_uid_, effect_uid_);
        assert(impl_ != nullptr);
      }
      return impl_;
    }
  };

private:
  /**
   * The reaction handler.
   *
   * This method is invoked automatically in response to triggering events. User
   * code must override this method to define a reaction's behavior.
   */
  virtual void handler() = 0;

  [[nodiscard]] static auto register_reaction(const ReactionProperties& properties, BaseReaction* self)
      -> std::uint64_t;

  std::uint64_t parent_uid_;
  std::optional<Duration> deadline_;

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

inline auto BaseReaction::register_reaction(const ReactionProperties& properties, BaseReaction* self) -> std::uint64_t {
  const Context context{properties.context_};
  return detail::register_with_location(context, [&]() -> std::uint64_t {
    auto& backend = detail::get_backend(context);
    auto parent = detail::ContextAccess::get_parent_uid(properties.context_);
    // Ownership of the handler transfers to the implementation on the
    // register call (including when it throws).
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    auto* handler = new HandlerImpl{*self};
    const std::string name{properties.name_};
    if (properties.deadline_.has_value()) {
      return backend.register_reaction_with_deadline(name, parent, handler, properties.position_,
                                                     *properties.deadline_);
    }
    return backend.register_reaction(name, parent, handler, properties.position_);
  });
}

inline auto BaseReaction::deadline() const noexcept -> std::optional<TimePoint> {
  if (!deadline_.has_value()) {
    return std::nullopt;
  }

  // The deadline can only be computed relative to a running program's logical
  // time. When called outside of execution (e.g. during reaction declaration),
  // there is no valid time access, so we report that no deadline is available.
  const auto* time_access = detail::get_reaction_time_access(*program_context(), parent_uid_);
  if (time_access == nullptr) {
    return std::nullopt;
  }

  auto now = time_access->get_timestamp();

  // guard against overflow
  if (TimePoint::max() - now < *deadline_) {
    return TimePoint::max();
  }

  return now + *deadline_;
}

} // namespace xronos::sdk

#endif // XRONOS_SDK_REACTION_HH
