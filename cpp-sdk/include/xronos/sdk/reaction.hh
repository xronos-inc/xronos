// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Class definitions related to reactions.
 */

#ifndef XRONOS_SDK_REACTION_HH
#define XRONOS_SDK_REACTION_HH

#include <cassert>
#include <string_view>
#include <type_traits>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/metric.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/time.hh"
#include "xronos/sdk/value_ptr.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/port.hh"

namespace xronos::sdk {
/**
 * @brief Opaque object used by reactions at construction time.
 *
 * @details This object can be used as arguments to the constructors of the
 * reaction's triggers, sources, and effects.
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
 * @cond Doxygen_Suppress
 * @internal
 * @brief Internal properties of a reaction.
 *
 * @details `ReactionProperties` objects should not be constructed or accessed
 * directly by application code.
 */
class ReactionProperties {
  constexpr ReactionProperties(std::string_view name, std::uint64_t reaction_id,
                               std::reference_wrapper<Reactor> container, ReactorContext context)
      : name_(name)
      , reaction_id_(reaction_id)
      , container_(container)
      , context_(context) {}

  std::string_view name_;
  std::uint64_t reaction_id_;
  std::reference_wrapper<Reactor> container_;
  ReactorContext context_;

  [[nodiscard]] auto container() noexcept -> Reactor& { return container_; }

  friend Reactor;
  friend BaseReaction;
  template <class R> friend class Reaction;
};
/**
 * @endcond
 */

/**
 * @brief Non-template base class for reactions.
 *
 * @details Application code should inherit from `Reaction` instead of directly
 * from `BaseReaction`.
 */
class BaseReaction : Element {
public:
  /**
   * @internal
   * @brief Constructor that should not be invoked directly by application code.
   *
   * @details Application code should instead use `Reaction::add_reaction` to
   * instantiate reactions.
   */
  BaseReaction(ReactionProperties properties);
  /**
   * @brief Correct deletion of an instance of a derived class is permitted.
   */
  virtual ~BaseReaction() = default;

protected:
  /**
   * @brief Get the context object that is needed to construct the reaction's
   * members.
   *
   * @return ReactionContext This reaction's initialization context.
   */
  [[nodiscard]] auto context() noexcept -> ReactionContext;

  template <class T> class Trigger;

  /**
   * @cond Doxygen_Suppress
   * @internal
   * @brief Access to a reactor element that this reaction may read.
   *
   * @details Unlike a `Trigger`, a `Source` does not trigger the reaction
   * handler to execute.
   * @tparam T The type of value carried by the source.
   */
  template <class T> class Source {
  public:
    /**
     * @internal
     * @brief Construct a new Source object.
     *
     * @param source An event source of the containing reactor, which can be
     * obtained using the `Reaction::self` method.
     * @param context The current reaction's initialization context, which can
     * be obtained using the `BaseReaction::context` method.
     */
    Source(const EventSource<T>& source, ReactionContext context)
        : Source<T>{source} {
      source.register_as_source_of(context.reaction_instance());
    }

    /**
     * @internal
     * @brief Get read access to the current value of the referenced reactor element.
     *
     * @return ImmutableValuePtr<T> A pointer to the current value of the reactor
     * element, or `null` if the element is not present.
     */
    [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T>
      requires(!std::is_same_v<T, void>)
    {
      return event_source_.get().get();
    }

    /**
     * @internal
     * @brief Check if the referenced reactor element is present at the current
     * timestamp.
     *
     * @return bool `true` if the element is present, `false` otherwise.
     */
    [[nodiscard]] auto is_present() const noexcept -> bool { return event_source_.get().is_present(); }

  private:
    Source(const EventSource<T>& event_source)
        : event_source_{event_source} {}
    std::reference_wrapper<const EventSource<T>> event_source_;

    friend BaseReaction::Trigger<T>;
  };
  /**
   * @endcond
   */

  /**
   * @brief Access to a reactor element that this reaction may write to.
   *
   * @details Unlike a `Source`, a `Trigger` triggers the reaction handler to
   * execute.
   * @tparam T The type of value carried by the trigger.
   */
  template <class T> class Trigger : public Source<T> {
  public:
    /**
     * @brief Construct a new Trigger object.
     *
     * @param trigger An event source of the containing reactor, which can be
     * obtained using the `Reaction::self` method.
     * @param context The current reaction's initialization context, which can
     * be obtained using the `BaseReaction::context` method.
     */
    Trigger(const EventSource<T>& trigger, ReactionContext context)
        : Source<T>{trigger} {
      trigger.register_as_trigger_of(context.reaction_instance());
    }
  };

  /**
   * @brief Access to a port that this reaction may write to.
   *
   * @tparam T The type of value carried by the port.
   */
  template <class T> class PortEffect {
  public:
    /**
     * @brief Construct a new `PortEffect` object.
     *
     * @param port An port of the containing reactor, which can be
     * obtained using the `Reaction::self` method.
     * @param context The current reaction's initialization context, which can
     * be obtained using the `BaseReaction::context` method.
     */
    PortEffect(Port<T>& port, ReactionContext context)
        : port_{port} {
      context.reaction_instance().declare_antidependency(&detail::get_runtime_instance<runtime::BasePort>(port));
    }

    /**
     * @brief Set the port value and send a message to connected ports.

     * @details Can be called multiple times, but at each time at most one value
     * is sent to connected ports. When called repeatedly at a given timestamp,
     * the previous value is overwritten.
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
     */
    template <class U>
    void set(const U& value)
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      set(make_immutable_value<T>(value));
    }
    /**
     * @overload
     */
    template <class U>
    void set(U&& value)
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      set(make_immutable_value<T>(std::forward<U>(value)));
    }

    /**
     * @brief Set the port without sending any value to connected ports.
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

  private:
    std::reference_wrapper<Port<T>> port_;
  };

  /**
   * @brief Access to a programmable timer that this reaction may schedule
   * events with.
   *
   * @tparam T The type of value carried by the programmable timer.
   */
  template <class T> class ProgrammableTimerEffect {
  public:
    /**
     * @brief Construct a new `ProgrammableTimerEffect` object.
     *
     * @param timer A programmable timer of the containing reactor, which can be
     * obtained using the `Reaction::self` method.
     * @param context The current reaction's initialization context, which can
     * be obtained using the `BaseReaction::context` method.
     */
    ProgrammableTimerEffect(ProgrammableTimer<T>& timer, ReactionContext context)
        : event_{timer} {
      context.reaction_instance().declare_schedulable_action(&detail::get_runtime_instance<runtime::BaseAction>(timer));
    }

    /**
     * @brief Schedule a future timer event.
     *
     * @param value_ptr The value to be associated with the event occurrence.
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
     */
    template <class U>
    void schedule(const U& value, Duration delay = Duration::zero())
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      schedule(make_immutable_value<T>(value), delay);
    }
    /**
     * @overload
     */
    template <class U>
    void schedule(U&& value, Duration delay = Duration::zero())
      requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
    {
      schedule(make_immutable_value<T>(std::forward<U>(value)), delay);
    }

    /**
     * @brief Schedule a future timer event.
     *
     * @param delay The time to wait until the new event is processed.
     */
    void schedule(Duration delay = Duration::zero())
      requires(std::is_same_v<T, void>)
    {
      event_.get().schedule(delay);
    }

    // Disambiguate set(0) by explicitly deleting set(nullptr_t)
    template <typename V>
    void schedule(V, Duration delay = Duration::zero())
      requires(std::is_same_v<V, std::nullptr_t>)
    = delete;

  private:
    std::reference_wrapper<ProgrammableTimer<T>> event_;
  };

  /**
   * @brief Access to a metric that can be recorded.
   */
  class MetricEffect {
  public:
    /**
     * @brief Construct a new `MetricEffect` object.
     *
     * @param metric A metric of the containing reactor, which can be obtained
     * using the `Reaction::self` method.
     * @param context The current reaction's initialization context, which can
     * be obtained using the `BaseReaction::context` method.
     */
    MetricEffect(Metric& metric, [[maybe_unused]] ReactionContext context)
        : metric_{metric} {}
    void record(double value) noexcept { metric_.get().record(value); }
    void record(std::int64_t value) noexcept { metric_.get().record(value); }

  private:
    std::reference_wrapper<Metric> metric_;
  };

private:
  /**
   * @brief The body of the reaction.
   *
   * @details This method should be overridden by application code to provide
   * the functionality of the reaction. This handler will be called by the
   * framework when the reaction is triggered.
   */
  virtual void handler() = 0;
};

/**
 * @brief Reaction base class that is intended to be inherited by application
 * code.
 *
 * @details Implementors should provide reaction functionality by overriding the
 * `BaseReaction::handler` method and should declare the triggers, sources, and
 * effects that the reaction depends on as members of the derived class.
 */
template <class R> class Reaction : public BaseReaction {
public:
  /**
   * @internal
   * @brief Constructor that should not be invoked directly by application code.
   *
   * @details Application code should instead use `Reactor::add_reaction` to
   * instantiate reactions.
   */
  Reaction(ReactionProperties properties)
      : BaseReaction(properties)
      , self_(dynamic_cast<R&>(properties.container())) {}

protected:
  /**
   * @brief Get a reference to the reaction's containing reactor.
   *
   * @details This reference is valid at reaction construction time.
   * @return R& A reference to the reaction's containing reactor.
   */
  [[nodiscard]] auto self() noexcept -> R& { return self_; }

private:
  std::reference_wrapper<R> self_;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_REACTION_HH
