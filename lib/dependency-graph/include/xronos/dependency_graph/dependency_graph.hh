// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_DEPENDENCY_GRAPH_DEPENDENCY_GRAPH_HH
#define XRONOS_DEPENDENCY_GRAPH_DEPENDENCY_GRAPH_HH

#include "nonstd/expected.hpp"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace xronos::dependency_graph {

struct WeakDependency {
  core::Duration delay;
  std::uint64_t reaction_uid;
};

struct StrongDependency {
  std::uint64_t reaction_uid;
};

class DependencyGraph {
public:
  [[nodiscard]] auto weak_dependencies() const noexcept -> const auto& { return weak_dependencies_; }
  [[nodiscard]] auto strong_dependencies() const noexcept -> const auto& { return strong_dependencies_; }

  void init(const core::ReactorModel& model);

  // Returns a total order of reaction uids according to their dependencies.
  // The first element in the returned list is the reaction to be executed
  // first.
  auto total_order(const core::ElementRegistry& elements) const
      -> nonstd::expected<std::vector<std::uint64_t>, std::string>;

private:
  void add_strong_dependency(std::uint64_t from_uid, std::uint64_t to_uid) {
    strong_dependencies_[from_uid].emplace_back(StrongDependency{to_uid});
  }

  void add_weak_dependency(std::uint64_t from_uid, std::uint64_t to_uid, core::Duration delay) {
    weak_dependencies_[from_uid].emplace_back(WeakDependency{.delay = delay, .reaction_uid = to_uid});
  }

  struct ReactionInfo {
    std::uint32_t pos;
    std::uint64_t uid;

    // Default comparison operator. Since `pos` is defined before `uid`, it will
    // sort reactions of the same reactor by their position.
    auto operator<=>(const ReactionInfo&) const = default;
  };

  void add_intra_rector_dependencies(const core::ElementRegistry& elements);
  void add_port_dependencies(const core::ReactorModel& model);

  bool initialized_{false};
  // key is a reaction uid
  std::unordered_map<std::uint64_t, std::vector<WeakDependency>> weak_dependencies_{};
  // key is a reaction uid
  std::unordered_map<std::uint64_t, std::vector<StrongDependency>> strong_dependencies_{};
};

} // namespace xronos::dependency_graph

#endif // XRONOS_DEPENDENCY_GRAPH_DEPENDENCY_GRAPH_HH
