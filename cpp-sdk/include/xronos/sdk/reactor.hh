// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_REACTOR_HH
#define XRONOS_SDK_REACTOR_HH

#include <cstdint>
#include <functional>
#include <memory>
#include <source_location>
#include <string_view>
#include <utility>
#include <vector>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/source_location.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/shutdown.hh"
#include "xronos/sdk/startup.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

namespace detail {

template <class ReactionClass>
  requires(std::is_base_of_v<BaseReaction, ReactionClass>)
auto add_reaction(Reactor& reactor, std::string_view name, detail::SourceLocationView source_location)
    -> ReactionClass&;

void runtime_connect(Reactor& reactor, const Element& from_port, const Element& to_port);
void runtime_connect(Reactor& reactor, const Element& from_port, const Element& to_port, Duration delay);

} // namespace detail

/**
 * An abstract reactor that can be subclassed to define new reactors.
 */
class Reactor : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the reactor instance.
   * @param parent_context Either the environment's or the containing reactor's context.
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
  /**
   * Get a context object for constructing reactor elements and contained
   * reactors.
   *
   * @param source_location Source location of the call site. Normally this
   * should be omitted to use the default argument.
   * @returns This reactors's context.
   */
  [[nodiscard]] auto context(std::source_location source_location = std::source_location::current()) noexcept
      -> ReactorContext;

  /**
   * Get the current timestamp.
   *
   * This does not read wall-clock time. The Xronos runtime uses an
   * internal clock to control how a program advances.
   *
   * @returns The current timestamp as provided by the internal clock.
   */
  [[nodiscard]] auto get_time() const noexcept -> TimePoint;

  /**
   * Get the current lag.
   *
   * Since in the Xronos SDK time does not advance while reactions execute, the
   * internal clock may advance slower than a wall clock would. The lag denotes
   * the difference between the wall clock and the internal clock. It is a
   * measure of how far the execution of reactions lags behind events in the
   * physical world.
   *
   * @returns The current lag.
   */
  [[nodiscard]] auto get_lag() const noexcept -> Duration;

  /**
   * Get the time that passed since the @ref startup event.
   *
   * @returns The difference between the current timestamp given by
   * get_time() and the timestamp at which the program started.
   */
  [[nodiscard]] auto get_time_since_startup() const noexcept -> Duration;

  /**
   * Get the startup event source.
   *
   * @returns An event source that triggers once when the program execution
   * starts.
   */
  [[nodiscard]] auto startup() const noexcept -> const EventSource<void>& { return startup_; }

  /**
   * Get the shutdown event source.
   *
   * @returns An event source that triggers once right before the program
   * execution ends.
   */
  [[nodiscard]] auto shutdown() const noexcept -> const EventSource<void>& { return shutdown_; }

  /**
   * @copydoc Environment::request_shutdown
   */
  void request_shutdown() noexcept;

  /**
   * @copydoc Environment::connect(const InputPort<T>&, const InputPort<T>&)
   *
   * @see assemble()
   */
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to) {
    detail::runtime_connect(*this, from, to);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to) {
    detail::runtime_connect(*this, from, to);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to) {
    detail::runtime_connect(*this, from, to);
  }

  /**
   * @copydoc Environment::connect(const InputPort<T>&, const InputPort<T>&, Duration)
   *
   * @see assemble()
   */
  template <class T> void connect(const InputPort<T>& from, const InputPort<T>& to, Duration delay) {
    detail::runtime_connect(*this, from, to, delay);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const OutputPort<T>& to, Duration delay) {
    detail::runtime_connect(*this, from, to, delay);
  }
  /**
   * @overload
   */
  template <class T> void connect(const OutputPort<T>& from, const InputPort<T>& to, Duration delay) {
    detail::runtime_connect(*this, from, to, delay);
  }

  /**
   * Instantiate and add a new reaction to the reactor.
   *
   * This is intended to be invoked when overloading assemble() in a subclass.
   * This factor method presents the only mechanism for correctly registering a
   * reaction for execution by the runtime.
   *
   * @tparam ReactionClass The reaction class to instantiate. This is typically a
   * subclass of Reaction.
   *
   * @param name The name of the reaction
   * @param source_location Source location of the call site. Normally this
   * should be omitted to use the default argument.
   *
   * @see assemble()
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
   * Method that sets up the internal topology of the reactor.
   *
   * This method needs to be overridden by concrete subclasses. The method is
   * called by the runtime before the execution starts. An implementation of
   * assemble() should call add_reaction() to instantiate the reactor's
   * reactions. If the reactor contains other reactors, their ports may be
   * connected using connect().
   */
  virtual void assemble() = 0;

  std::vector<std::unique_ptr<BaseReaction>> reactions_{};

  friend BaseReaction;
  friend auto detail::create_context(Reactor& reactor, detail::SourceLocationView source_location) -> ReactorContext;
  template <class ReactionClass>
    requires(std::is_base_of_v<BaseReaction, ReactionClass>)
  friend auto detail::add_reaction(Reactor& reactor, std::string_view name, detail::SourceLocationView source_location)
      -> ReactionClass&;
};

/**
 * Opaque data type used for constructing reactions.
 *
 * May not be constructed directly.
 *
 * @see Reactor::add_reaction()
 * @see BaseReaction::BaseReaction()
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
  friend auto detail::add_reaction(Reactor& reactor, std::string_view name, detail::SourceLocationView source_location)
      -> ReactionClass&;
};

} // namespace xronos::sdk

#include "xronos/sdk/reaction.hh"

namespace xronos::sdk::detail {

template <class ReactionClass>
  requires(std::is_base_of_v<BaseReaction, ReactionClass>)
auto add_reaction(Reactor& reactor, std::string_view name, detail::SourceLocationView source_location)
    -> ReactionClass& {
  auto reaction = std::make_unique<ReactionClass>(ReactionProperties{name, reactor.reactions_.size() + 1, reactor,
                                                                     detail::create_context(reactor, source_location)});
  auto& ref = *reaction;
  reactor.reactions_.emplace_back(std::move(reaction));
  return ref;
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_REACTOR_HH
