// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `Reactor` class.
 */

#ifndef XRONOS_SDK_REACTOR_HH
#define XRONOS_SDK_REACTOR_HH

#include <memory>
#include <source_location>
#include <string_view>
#include <utility>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/shutdown.hh"
#include "xronos/sdk/startup.hh"
#include "xronos/sdk/time.hh"

#include "xronos/runtime/connection_properties.hh"
#include "xronos/runtime/reactor.hh"

namespace xronos::sdk {

namespace detail {

template <class ReactionClass>
  requires(std::is_base_of_v<BaseReaction, ReactionClass>)
auto add_reaction(Reactor& reactor, std::string_view name,
                  detail::SourceLocationView source_location) -> ReactionClass&;

} // namespace detail

/**
 * @brief An abstract reactor that can be subclassed to define new reactors.
 */
class Reactor : public Element {
public:
  /**
   * @brief Construct a new `Reactor` object.
   *
   * @param name The name of the reactor instance.
   * @param parent_context The initialization context of the reactor's parent
   * reactor or environment, typically obtained from `Reactor::context` or
   * `Environment::context`.
   */
  Reactor(std::string_view name, Context parent_context);

  // Reactors may not be moved or copied. Copying is implicitly deleted due to
  // the unique_ptr to the runtime instance in Element. We also need to delete
  // the move constructor and assignment operator to avoid dangling references
  // in the ReactorContext objects.
  Reactor(Reactor&&) = delete;
  Reactor(const Reactor&) = delete;
  auto operator=(Reactor&&) = delete;
  auto operator=(const Reactor&) = delete;
  ~Reactor() override = default;

protected:
  [[nodiscard]] auto
  context(std::source_location source_location = std::source_location::current()) noexcept -> ReactorContext;

  /**
   * @brief Get the current point in time.
   *
   * @details This does not read wall-clock time. The `xronos` runtime uses an
   * internal clock to control how a program advances. `Reactor::get_time` reads the
   * current time as measured by the internal clock.
   * @return TimePoint The current point in time.
   */
  [[nodiscard]] auto get_time() const noexcept -> TimePoint;
  /**
   * @brief Get the current lag.
   *
   * @details Since time in the `xronos` runtime does not advance while
   * reactions execute, the internal clock may advance slower than a wall clock
   * would. The lag denotes the difference between the wall clock and the
   * internal clock. It is a measure of how far the execution of reactions lags
   * behind events in the physical world.
   * @return Duration The current lag.
   */
  [[nodiscard]] auto get_lag() const noexcept -> Duration;
  /**
   * @brief Get the time that passed since the `startup` event.
   *
   * @return Duration The difference between the current time point given by
   * `Reactor::get_time` and the time at which the program started.
   */
  [[nodiscard]] auto get_time_since_startup() const noexcept -> Duration;

  /**
   * @brief Startup event.
   *
   * @return const EventSource<void>& The event that triggers once when the
   * program execution starts.
   */
  [[nodiscard]] auto startup() const noexcept -> const EventSource<void>& { return startup_; }
  /**
   * @brief Shutdown event.
   *
   * @return const EventSource<void>& The event that triggers once right before
   * the program execution ends.
   */
  [[nodiscard]] auto shutdown() const noexcept -> const EventSource<void>& { return shutdown_; }

  /**
   * @brief Request the termination of the currently running reactor program.
   *
   * @details Terminates a program started with `Environment::execute` at the next
   * convenience. This triggers `Reactor::shutdown` after completing all
   * currently active reactions, and stops program execution after processing
   * all reactions triggered by `Reactor::shutdown`.
   */
  void request_shutdown() noexcept;

  /**
   * @brief Connect two ports.
   *
   * @details Creates a new connection from the port given in @p from to the
   * port given in @p to.
   * @details This is intended to be invoked in `Reactor::assemble`.
   * @tparam T The type carried by the connection.
   * @param from The port to draw the connection from.
   * @param to The port to draw the connection to.
   */
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to) {
    try {
      detail::get_runtime_instance<runtime::Reactor>(*this).environment().draw_connection(
          detail::get_runtime_instance<runtime::Port<T>>(from), detail::get_runtime_instance<runtime::Port<T>>(to), {});
    } catch (const runtime::ValidationError& e) {
      throw ValidationError(e.what());
    }
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to) {
    try {
      detail::get_runtime_instance<runtime::Reactor>(*this).environment().draw_connection(
          detail::get_runtime_instance<runtime::Port<T>>(from), detail::get_runtime_instance<runtime::Port<T>>(to), {});
    } catch (const runtime::ValidationError& e) {
      throw ValidationError(e.what());
    }
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to) {
    try {
      detail::get_runtime_instance<runtime::Reactor>(*this).environment().draw_connection(
          detail::get_runtime_instance<runtime::Port<T>>(from), detail::get_runtime_instance<runtime::Port<T>>(to), {});
    } catch (const runtime::ValidationError& e) {
      throw ValidationError(e.what());
    }
  }
  /**
   * @brief Connect two ports with a delay.
   *
   * @details Creates a new connection from the port given in @p from to the port
   * given in @p to.
   * @details This is intended to be invoked in `Reactor::assemble`.
   * @tparam T The type carried by the connection.
   * @param from The port to draw the connection from.
   * @param to The port to draw the connection to.
   * @param delay The connection waits for `delay` before delivering the messages
   * to @p to.
   */
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to, Duration delay) {
    try {
      detail::get_runtime_instance<runtime::Reactor>(*this).environment().draw_connection(
          detail::get_runtime_instance<runtime::Port<T>>(from), detail::get_runtime_instance<runtime::Port<T>>(to),
          {runtime::ConnectionType::Delayed, delay});
    } catch (const runtime::ValidationError& e) {
      throw ValidationError(e.what());
    }
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to, Duration delay) {
    try {
      detail::get_runtime_instance<runtime::Reactor>(*this).environment().draw_connection(
          detail::get_runtime_instance<runtime::Port<T>>(from), detail::get_runtime_instance<runtime::Port<T>>(to),
          {runtime::ConnectionType::Delayed, delay});
    } catch (const runtime::ValidationError& e) {
      throw ValidationError(e.what());
    }
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to, Duration delay) {
    try {
      detail::get_runtime_instance<runtime::Reactor>(*this).environment().draw_connection(
          detail::get_runtime_instance<runtime::Port<T>>(from), detail::get_runtime_instance<runtime::Port<T>>(to),
          {runtime::ConnectionType::Delayed, delay});
    } catch (const runtime::ValidationError& e) {
      throw ValidationError(e.what());
    }
  }

  /**
   * @brief Instantiate and add a new reaction to the reactor.
   *
   * @details This is intended to be invoked in `Reactor::assemble` and is the
   * only way to correctly register a reaction for execution by the runtime.
   * @tparam ReactionClass The reaction to instantiate. This is typically a
   * subclass of `Reaction`.
   * @param name The name of the reaction
   * @param source_location Source location of the call site. Normally this
   * should be omitted to use the default argument.
   */
  template <class ReactionClass>
    requires(std::is_base_of_v<BaseReaction, ReactionClass>)
  void add_reaction(std::string_view name, std::source_location source_location = std::source_location::current()) {
    detail::add_reaction<ReactionClass>(*this, name, detail::SourceLocationView::from_std(source_location));
  }

private:
  std::reference_wrapper<Environment> environment_;

  Startup startup_;
  Shutdown shutdown_;

  /**
   * @brief Method that sets up the internal topology of the reactor.
   *
   * @details This method should be overridden by application code so that the
   * `xronos` framework can correctly connect ports and instantiate reactions
   * during program initialization.
   */
  virtual void assemble() = 0;

  std::vector<std::unique_ptr<BaseReaction>> reactions_{};

  friend BaseReaction;
  friend auto detail::create_context(Reactor& reactor, detail::SourceLocationView source_location) -> ReactorContext;
  template <class ReactionClass>
    requires(std::is_base_of_v<BaseReaction, ReactionClass>)
  friend auto detail::add_reaction(Reactor& reactor, std::string_view name,
                                   detail::SourceLocationView source_location) -> ReactionClass&;
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

  friend BaseReaction;
  template <class R> friend class Reaction;
  template <class ReactionClass>
    requires(std::is_base_of_v<BaseReaction, ReactionClass>)
  friend auto detail::add_reaction(Reactor& reactor, std::string_view name,
                                   detail::SourceLocationView source_location) -> ReactionClass&;
};
/**
 * @endcond
 */

} // namespace xronos::sdk

#include "xronos/sdk/reaction.hh"

namespace xronos::sdk::detail {

template <class ReactionClass>
  requires(std::is_base_of_v<BaseReaction, ReactionClass>)
auto add_reaction(Reactor& reactor, std::string_view name,
                  detail::SourceLocationView source_location) -> ReactionClass& {
  auto reaction = std::make_unique<ReactionClass>(ReactionProperties{name, reactor.reactions_.size() + 1, reactor,
                                                                     detail::create_context(reactor, source_location)});
  auto& ref = *reaction;
  reactor.reactions_.emplace_back(std::move(reaction));
  return ref;
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_REACTOR_HH
