// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_ACTION_HH
#define XRONOS_RUNTIME_ACTION_HH

#include "assert.hh"
#include "environment.hh"
#include "fwd.hh"
#include "logical_time.hh"
#include "reactor.hh"
#include "reactor_element.hh"
#include "time.hh"
#include "time_barrier.hh"
#include "value_ptr.hh"

#include <condition_variable>
#include <map>
#include <mutex>

namespace xronos::runtime {

class BaseAction : public ReactorElement {
private:
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> schedulers_{};
  Duration min_delay_{0};
  bool logical_{true};
  bool present_{false};

protected:
  void register_trigger(Reaction* reaction);
  void register_scheduler(Reaction* reaction);

  virtual void setup() noexcept { present_ = true; }
  virtual void cleanup() noexcept { present_ = false; }

  /**
   * Use the given condition variable and lock to wait until the given tag it
   * safe to process. The waiting is interrupted when the condition variable is
   * notified (or has a spurious wakeup) and a call to the given `abort_waiting`
   * function returns true. or until the condition variable is notified.
   *
   * Returns false if the wait was interrupted and true otherwise. True
   * indicates that the tag is safe to process.
   */
  virtual auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                           const std::function<bool(void)>& abort_waiting) -> bool;

  BaseAction(std::string_view name, Reactor& container, bool logical, Duration min_delay)
      : ReactorElement(name, container)
      , min_delay_(min_delay)
      , logical_(logical) {}
  BaseAction(std::string_view name, Environment& environment, bool logical, Duration min_delay);

public:
  [[nodiscard]] auto triggers() const noexcept -> const auto& { return triggers_; }
  [[nodiscard]] auto schedulers() const noexcept -> const auto& { return schedulers_; }
  [[nodiscard]] auto is_logical() const noexcept -> bool { return logical_; }
  [[nodiscard]] auto is_physical() const noexcept -> bool { return !logical_; }
  [[nodiscard]] auto min_delay() const noexcept -> Duration { return min_delay_; }
  [[nodiscard]] auto is_present() const noexcept -> bool { return present_; }

  [[nodiscard]] auto element_type() const -> std::string_view override { return "action"; };
  void visit(ReactorElementVisitor& visitor) const override { visitor.visit(*this); };

  friend class Reaction;
  friend class Scheduler;
};

template <class T> class Action : public BaseAction {
private:
  ImmutableValuePtr<T> value_ptr_{nullptr};

  std::map<Tag, ImmutableValuePtr<T>> events_{};
  std::mutex mutex_events_{};

protected:
  void setup() noexcept override;
  void cleanup() noexcept final;

  Action(std::string_view name, Reactor& container, bool logical, Duration min_delay)
      : BaseAction(name, container, logical, min_delay) {}
  Action(std::string_view name, Environment& environment, bool logical, Duration min_delay)
      : BaseAction(name, environment, logical, min_delay) {}

public:
  // Normally, we should lock the mutex while moving to make this
  // fully thread safe. However, we rely assembly happening before
  // execution and hence can ignore the mutex.
  Action(Action&& action) noexcept
      : BaseAction(std::move(action)) {}
  auto operator=(Action&& action) noexcept -> Action& {
    BaseAction::operator=(std::move(action));
    return *this;
  }

  Action(const Action& action) = delete;
  auto operator=(const Action& action) -> Action& = delete;

  ~Action() override = default;

  void startup() final {}
  void shutdown() final {}

  template <class Dur = Duration> void schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay = Dur::zero());
  auto schedule_at(const ImmutableValuePtr<T>& value_ptr, const Tag& tag) -> bool;

  template <class Dur = Duration> void schedule(MutableValuePtr<T>&& value_ptr, Dur delay = Dur::zero()) {
    schedule(ImmutableValuePtr<T>(std::move(value_ptr)), delay);
  }

  template <class Dur = Duration> void schedule(const T& value, Dur delay = Dur::zero()) {
    schedule(make_immutable_value<T>(value), delay);
  }

  template <class Dur = Duration> void schedule(T&& value, Dur delay = Dur::zero()) {
    schedule(make_immutable_value<T>(std::move(value)), delay);
  }

  // Scheduling an action with nullptr value is not permitted.
  template <class Dur = Duration> void schedule(std::nullptr_t, Dur) = delete;

  [[nodiscard]] auto get() const noexcept -> const ImmutableValuePtr<T>& { return value_ptr_; }
};

template <> class Action<void> : public BaseAction {
protected:
  Action(std::string_view name, Reactor& container, bool logical, Duration min_delay)
      : BaseAction(name, container, logical, min_delay) {}
  Action(std::string_view name, Environment& environment, bool logical, Duration min_delay)
      : BaseAction(name, environment, logical, min_delay) {}

public:
  template <class Dur = Duration> void schedule(Dur delay = Dur::zero());
  auto schedule_at(const Tag& tag) -> bool;

  void startup() final {}
  void shutdown() final {}
};

template <class T> class PhysicalAction : public Action<T> {
public:
  PhysicalAction(std::string_view name, Reactor& container);
};

template <class T> class LogicalAction : public Action<T> {
public:
  LogicalAction(std::string_view name, Reactor& container, Duration min_delay = Duration::zero())
      : Action<T>(name, container, true, min_delay) {}
};

class Timer : public BaseAction {
private:
  Duration offset_{0};
  Duration period_{0};

  void cleanup() noexcept final;

public:
  Timer(std::string_view name, Reactor& container, Duration period = Duration::zero(),
        Duration offset = Duration::zero())
      : BaseAction(name, container, true, Duration::zero())
      , offset_(offset)
      , period_(period) {}

  void startup() final;
  void shutdown() final {}

  [[nodiscard]] auto element_type() const -> std::string_view override { return "timer"; };
  void visit(ReactorElementVisitor& visitor) const override { visitor.visit(*this); };

  [[nodiscard]] auto offset() const noexcept -> const Duration& { return offset_; }
  [[nodiscard]] auto period() const noexcept -> const Duration& { return period_; }

  void set_offset(Duration offset);
  void set_period(Duration period);
};

class StartupTrigger : public BaseAction {
public:
  StartupTrigger(std::string_view name, Reactor& container)
      : BaseAction(name, container, true, Duration::zero()) {}
  [[nodiscard]] auto element_type() const -> std::string_view final { return "startup"; };
  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  void startup() final;
  void shutdown() final {}
};

class ShutdownTrigger : public BaseAction {
public:
  ShutdownTrigger(std::string_view name, Reactor& container)
      : BaseAction(name, container, true, Duration::zero()) {}

  [[nodiscard]] auto element_type() const -> std::string_view final { return "shutdown"; };
  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  void startup() final;
  void shutdown() final;
};

} // namespace xronos::runtime

#include "impl/action_impl.hh"

#endif // XRONOS_RUNTIME_ACTION_HH
