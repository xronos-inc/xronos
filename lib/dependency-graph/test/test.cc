// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// These tests check the reaction ordering edges the dependency graph derives
// from connections and from direct port access, and the cycle diagnostics of
// total_order.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/dependency_graph/dependency_graph.hh"

namespace {

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;
namespace core = xronos::core;
using xronos::dependency_graph::DependencyGraph;

auto add_reactor(core::ReactorModel& model, std::string_view name,
                 std::optional<std::uint64_t> parent_uid = std::nullopt) -> std::uint64_t {
  return model.element_registry.add_new_element(name, core::ReactorTag{}, parent_uid).value().get().uid;
}

auto add_reaction(core::ReactorModel& model, std::uint64_t reactor_uid, std::string_view name,
                  std::uint32_t position = 0) -> std::uint64_t {
  auto properties = std::make_unique<core::ReactionProperties>();
  properties->position = position;
  return model.element_registry.add_new_element(name, core::ReactionTag{std::move(properties)}, reactor_uid)
      .value()
      .get()
      .uid;
}

auto add_input_port(core::ReactorModel& model, std::uint64_t reactor_uid, std::string_view name) -> std::uint64_t {
  return model.element_registry
      .add_new_element(name, core::InputPortTag{std::make_unique<core::PortProperties>()}, reactor_uid)
      .value()
      .get()
      .uid;
}

auto add_output_port(core::ReactorModel& model, std::uint64_t reactor_uid, std::string_view name) -> std::uint64_t {
  return model.element_registry
      .add_new_element(name, core::OutputPortTag{std::make_unique<core::PortProperties>()}, reactor_uid)
      .value()
      .get()
      .uid;
}

// A reactor with one reaction that triggers on an input port and writes an
// output port with zero delay.
struct ForwardingReactor {
  std::uint64_t reactor;
  std::uint64_t reaction;
  std::uint64_t input;
  std::uint64_t output;
};

auto add_forwarding_reactor(core::ReactorModel& model, std::string_view name,
                            std::optional<std::uint64_t> parent_uid = std::nullopt) -> ForwardingReactor {
  auto reactor = add_reactor(model, name, parent_uid);
  auto reaction = add_reaction(model, reactor, "forward");
  auto input = add_input_port(model, reactor, "input");
  auto output = add_output_port(model, reactor, "output");
  model.reaction_dependency_registry.register_reaction_trigger(reaction, input);
  model.reaction_dependency_registry.register_reaction_effect(reaction, output);
  return {.reactor = reactor, .reaction = reaction, .input = input, .output = output};
}

void connect(core::ReactorModel& model, std::uint64_t from_uid, std::uint64_t to_uid,
             std::optional<core::Duration> delay = std::nullopt) {
  bool added = model.connection_graph.add_connection(core::ConnectionProperties{
      .from_uid = from_uid, .to_uid = to_uid, .delay = delay, .crossing = core::BoundaryCrossing::None});
  REQUIRE(added);
}

auto index_of(const std::vector<std::uint64_t>& order, std::uint64_t uid) -> std::size_t {
  auto it = std::ranges::find(order, uid);
  REQUIRE(it != order.end());
  return static_cast<std::size_t>(it - order.begin());
}

TEST_CASE("Direct port access orders the writer before the reader", "[dependency-graph]") {
  core::ReactorModel model{};
  auto parent = add_reactor(model, "parent");
  auto writer = add_reaction(model, parent, "writer");
  auto child = add_forwarding_reactor(model, "child", parent);
  model.reaction_dependency_registry.register_reaction_effect(writer, child.input);

  DependencyGraph graph{};
  graph.init(model);
  auto order = graph.total_order(model.element_registry);

  REQUIRE(order.has_value());
  CHECK(index_of(*order, writer) < index_of(*order, child.reaction));
}

TEST_CASE("A zero-delay cycle through direct port access is rejected", "[dependency-graph]") {
  core::ReactorModel model{};
  auto parent = add_reactor(model, "parent");
  auto child = add_forwarding_reactor(model, "child", parent);
  auto reaction = add_reaction(model, parent, "loop_back");
  model.reaction_dependency_registry.register_reaction_trigger(reaction, child.output);
  model.reaction_dependency_registry.register_reaction_effect(reaction, child.input);

  DependencyGraph graph{};
  graph.init(model);
  auto order = graph.total_order(model.element_registry);

  REQUIRE_FALSE(order.has_value());
  CHECK_THAT(order.error(), ContainsSubstring("parent.loop_back"));
  CHECK_THAT(order.error(), ContainsSubstring("parent.child.forward"));
  CHECK_THAT(order.error(), ContainsSubstring("direct accesses"));
  CHECK_THAT(order.error(), ContainsSubstring("parent.child.input"));
  CHECK_THAT(order.error(), ContainsSubstring("parent.child.output"));
}

TEST_CASE("A delayed connection breaks a cycle through direct port access", "[dependency-graph]") {
  core::ReactorModel model{};
  auto parent = add_reactor(model, "parent");
  auto child = add_forwarding_reactor(model, "child", parent);
  auto reaction = add_reaction(model, parent, "loop_back");
  auto loop_out = add_output_port(model, parent, "loop_out");
  model.reaction_dependency_registry.register_reaction_trigger(reaction, child.output);
  model.reaction_dependency_registry.register_reaction_effect(reaction, loop_out);
  connect(model, loop_out, child.input, 1ms);

  DependencyGraph graph{};
  graph.init(model);
  auto order = graph.total_order(model.element_registry);

  REQUIRE(order.has_value());
  CHECK(index_of(*order, child.reaction) < index_of(*order, reaction));
}

TEST_CASE("A reaction that triggers on and effects the same port is rejected", "[dependency-graph]") {
  core::ReactorModel model{};
  auto parent = add_reactor(model, "parent");
  auto child = add_forwarding_reactor(model, "child", parent);
  auto reaction = add_reaction(model, parent, "self_loop");
  model.reaction_dependency_registry.register_reaction_trigger(reaction, child.input);
  model.reaction_dependency_registry.register_reaction_effect(reaction, child.input);

  DependencyGraph graph{};
  graph.init(model);
  auto order = graph.total_order(model.element_registry);

  REQUIRE_FALSE(order.has_value());
  CHECK_THAT(order.error(), ContainsSubstring("parent.self_loop"));
  CHECK_THAT(order.error(), ContainsSubstring("parent.child.input"));
}

TEST_CASE("A shared timer imposes no same-tag order", "[dependency-graph]") {
  core::ReactorModel model{};
  auto reactor = add_reactor(model, "reactor");
  auto timer = model.element_registry.add_new_element("timer", core::ProgrammableTimerTag{}, reactor).value().get().uid;
  auto reaction = add_reaction(model, reactor, "schedule_and_react");
  model.reaction_dependency_registry.register_reaction_trigger(reaction, timer);
  model.reaction_dependency_registry.register_reaction_effect(reaction, timer);

  DependencyGraph graph{};
  graph.init(model);
  auto order = graph.total_order(model.element_registry);

  REQUIRE(order.has_value());
}

} // namespace
