// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_IMPL_ACTION_HH
#define XRONOS_RUNTIME_DEFAULT_IMPL_ACTION_HH

#include <any>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string_view>
#include <utility>
#include <variant>

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/fwd.hh"
#include "xronos/runtime/default/impl/logical_time.hh"
#include "xronos/runtime/default/impl/reactor.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_::impl {

class BaseAction : public ReactorElement, public GettableTrigger {
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

  BaseAction(const core::Element& element_info, Reactor& container, bool logical, Duration min_delay)
      : ReactorElement(element_info, container)
      , min_delay_(min_delay)
      , logical_(logical) {}

public:
  [[nodiscard]] auto triggers() const noexcept -> const auto& { return triggers_; }
  [[nodiscard]] auto schedulers() const noexcept -> const auto& { return schedulers_; }
  [[nodiscard]] auto is_logical() const noexcept -> bool { return logical_; }
  [[nodiscard]] auto is_physical() const noexcept -> bool { return !logical_; }
  [[nodiscard]] auto min_delay() const noexcept -> Duration { return min_delay_; }

  [[nodiscard]] auto is_present() const noexcept -> bool final { return present_; }
  [[nodiscard]] auto get() const noexcept -> std::any override { return std::monostate{}; }

  [[nodiscard]] auto element_type() const -> std::string_view override { return "action"; };
  void visit(ReactorElementVisitor& visitor) const override { visitor.visit(*this); };

  friend class Reaction;
  friend class Scheduler;
};

class Action : public BaseAction, public SchedulableEffect {
private:
  std::any current_value_{};

  std::map<Tag, std::any> events_{};
  std::mutex mutex_events_{};

protected:
  void setup() noexcept override;
  void cleanup() noexcept final;

  using BaseAction::BaseAction;

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

  auto schedule_at(const std::any& value, const Tag& tag) -> bool;

  void schedule(const std::any& value, Duration delay = Duration::zero()) noexcept final;

  [[nodiscard]] auto get() const noexcept -> std::any final { return current_value_; }
};

class PhysicalAction : public Action, public ExternalTrigger {
public:
  PhysicalAction(const core::Element& element_info, Reactor& container);

  void trigger(const std::any& value) noexcept final { schedule(value); };
};

class LogicalAction : public Action {
public:
  LogicalAction(const core::Element& element_info, Reactor& container, Duration min_delay = Duration::zero())
      : Action(element_info, container, true, min_delay) {}
};

class Timer : public BaseAction {
private:
  Duration offset_{0};
  Duration period_{0};

  void cleanup() noexcept final;

public:
  Timer(const core::Element& element_info, Reactor& container);

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
  StartupTrigger(const core::Element& element_info, Reactor& container)
      : BaseAction(element_info, container, true, Duration::zero()) {}
  [[nodiscard]] auto element_type() const -> std::string_view final { return "startup"; };
  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  void startup() final;
  void shutdown() final {}
};

class ShutdownTrigger : public BaseAction {
public:
  ShutdownTrigger(const core::Element& element_info, Reactor& container)
      : BaseAction(element_info, container, true, Duration::zero()) {}

  [[nodiscard]] auto element_type() const -> std::string_view final { return "shutdown"; };
  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  void startup() final;
  void shutdown() final;
};

} // namespace xronos::runtime::default_::impl

#endif // XRONOS_RUNTIME_DEFAULT_IMPL_ACTION_HH
