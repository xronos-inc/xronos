// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_REACTION_HH
#define XRONOS_RUNTIME_REACTION_HH

#include <cstdint>
#include <functional>
#include <set>
#include <string_view>

#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/runtime/time.hh"

namespace xronos::runtime {

class Reaction : public ReactorElement { // NOLINT(cppcoreguidelines-special-member-functions)
private:
  std::set<BaseAction*> action_triggers_;
  std::set<BaseAction*> scheduable_actions_;
  std::set<Port*> port_trigger_;
  std::set<Port*> antidependencies_;
  std::set<Port*> dependencies_;
  std::set<BaseAction*> action_dependencies_;

  std::uint32_t priority_;
  std::uint32_t index_{};

  std::function<void(void)> body_{nullptr};

  Duration deadline_{Duration::zero()};
  std::function<void(void)> deadline_handler_{nullptr};

  void set_deadline_impl(Duration deadline, const std::function<void(void)>& handler);

public:
  Reaction(std::string_view name, std::uint32_t priority, Reactor& container, std::function<void(void)> body);

  ~Reaction() override = default;

  void declare_trigger(BaseAction* action);
  void declare_trigger(Port* port);
  void declare_schedulable_action(BaseAction* action);
  void declare_antidependency(Port* port);
  void declare_dependency(Port* port);
  void declare_dependency(BaseAction* action);

  [[nodiscard]] auto action_triggers() const noexcept -> const auto& { return action_triggers_; }

  [[nodiscard]] auto port_triggers() const noexcept -> const auto& { return port_trigger_; }

  [[maybe_unused]] [[nodiscard]] auto antidependencies() const noexcept -> const auto& { return antidependencies_; }

  [[nodiscard]] auto dependencies() const noexcept -> const auto& { return dependencies_; }
  [[nodiscard]] auto action_dependencies() const noexcept -> const auto& { return action_dependencies_; }

  [[maybe_unused]] [[nodiscard]] auto scheduable_actions() const noexcept -> const auto& { return scheduable_actions_; }

  [[nodiscard]] auto priority() const noexcept -> auto { return priority_; }

  [[nodiscard]] auto deadline() const noexcept -> auto { return deadline_; }

  void startup() final {}
  void shutdown() final {}

  [[nodiscard]] auto element_type() const -> std::string_view final { return "reaction"; };
  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  void trigger();
  void set_index(unsigned index);

  template <class Dur> void set_deadline(Dur deadline, const std::function<void(void)>& handler) {
    set_deadline_impl(std::chrono::duration_cast<Duration>(deadline), handler);
  }

  [[nodiscard]] auto has_deadline() const noexcept -> bool { return deadline_ != Duration::zero(); }

  [[nodiscard]] auto index() const noexcept -> auto { return index_; }
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_REACTION_HH
