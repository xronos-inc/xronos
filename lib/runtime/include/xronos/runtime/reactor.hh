// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_REACTOR_HH
#define XRONOS_RUNTIME_REACTOR_HH

#include <memory>
#include <set>
#include <string_view>
#include <vector>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/logical_time.hh"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/runtime/time.hh"

namespace xronos::runtime {
class Reactor : public ReactorElement {

private:
  std::set<ReactorElement*> elements_{};
  // Keep a dedicated list of contained reactors and reactions, as we need to
  // access them separately for traversing the instance tree and for analyzing
  // the reaction dependencies during initialization.
  std::vector<Reactor*> contained_reactors_{};
  std::vector<Reaction*> reactions_{};
  // A list of connection objects owned by this reactor. While the connections
  // are created by the Environment, this list is used to keep track of the
  // connections and keep them alive. They are only deleted once the reactor is
  // deconstructed.
  std::vector<std::unique_ptr<BaseAction>> connections_{};

  void register_contained_reactor(Reactor& reactor);

public:
  Reactor(std::string_view name, Reactor& container);
  Reactor(std::string_view name, Environment& environment);
  ~Reactor() override = default;

  void register_element(ReactorElement& element);
  void register_reaction(Reaction& reaction);
  void add_connection(std::unique_ptr<BaseAction> connection);

  [[nodiscard]] auto elements() const noexcept -> const auto& { return elements_; }
  [[nodiscard]] auto connections() const noexcept -> const auto& { return connections_; }
  [[nodiscard]] auto contained_reactors() const noexcept -> const auto& { return contained_reactors_; }
  [[nodiscard]] auto reactions() const noexcept -> const auto& { return reactions_; }

  void startup() final;
  void shutdown() final;

  [[nodiscard]] auto element_type() const -> std::string_view final { return "reactor"; };

  void visit(ReactorElementVisitor& visitor) const final { visitor.visit(*this); };

  virtual void assemble() = 0;

  [[nodiscard]] static auto get_physical_time() noexcept -> TimePoint;
  [[nodiscard]] auto get_logical_time() const noexcept -> TimePoint;
  [[nodiscard]] auto get_microstep() const noexcept -> mstep_t;
  [[nodiscard]] auto get_tag() const noexcept -> Tag;
  [[nodiscard]] auto get_elapsed_logical_time() const noexcept -> Duration;
  [[nodiscard]] auto get_elapsed_physical_time() const noexcept -> Duration;
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_REACTOR_HH
