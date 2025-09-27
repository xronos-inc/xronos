// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_IMPL_REACTOR_HH
#define XRONOS_RUNTIME_DEFAULT_IMPL_REACTOR_HH

#include <cstdint>
#include <memory>
#include <set>
#include <string_view>
#include <vector>

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/fwd.hh"
#include "xronos/runtime/default/impl/logical_time.hh"
#include "xronos/runtime/default/impl/reactor_element.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_::impl {

class Reactor : public ReactorElement, public TimeAccess {

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
  Reactor(const core::Element& element_info, Reactor& container);
  Reactor(const core::Element& element_info, Environment& environment);
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

  [[nodiscard]] static auto get_physical_time() noexcept -> TimePoint;
  [[nodiscard]] auto get_logical_time() const noexcept -> TimePoint;
  [[nodiscard]] auto get_tag() const noexcept -> Tag;
  [[nodiscard]] auto get_elapsed_logical_time() const noexcept -> Duration;
  [[nodiscard]] auto get_elapsed_physical_time() const noexcept -> Duration;

  [[nodiscard]] auto get_timestamp() const noexcept -> core::TimePoint final { return get_logical_time(); }
  [[nodiscard]] auto get_microstep() const noexcept -> std::uint32_t final;
  [[nodiscard]] auto get_start_timestamp() const noexcept -> core::TimePoint final {
    return environment().start_tag().time_point();
  }
};

} // namespace xronos::runtime::default_::impl

#endif // XRONOS_RUNTIME_DEFAULT_IMPL_REACTOR_HH
