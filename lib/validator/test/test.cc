// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// These tests enumerate the shapes the hierarchy rules allow and reject for
// connections, reaction triggers, and reaction effects.

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/validator/checks.hh"
#include "gtest/gtest.h"

namespace {

namespace core = xronos::core;
namespace validator = xronos::validator;

// Builds elements in a shared model and wraps the per-edge checks with
// EXPECT-friendly predicates.
class HierarchyCheck : public ::testing::Test {
protected:
  auto add(std::string_view name, core::ElementType type, std::optional<std::uint64_t> parent = std::nullopt)
      -> std::uint64_t {
    return model_.element_registry.add_new_element(name, std::move(type), parent).value().get().uid;
  }

  auto add_reactor(std::string_view name, std::optional<std::uint64_t> parent = std::nullopt) -> std::uint64_t {
    return add(name, core::ReactorTag{}, parent);
  }
  auto add_reaction(std::uint64_t reactor, std::string_view name = "reaction") -> std::uint64_t {
    return add(name, core::ReactionTag{std::make_unique<core::ReactionProperties>()}, reactor);
  }
  auto add_input(std::uint64_t reactor, std::string_view name = "input") -> std::uint64_t {
    return add(name, core::InputPortTag{std::make_unique<core::PortProperties>()}, reactor);
  }
  auto add_output(std::uint64_t reactor, std::string_view name = "output") -> std::uint64_t {
    return add(name, core::OutputPortTag{std::make_unique<core::PortProperties>()}, reactor);
  }

  auto connection_valid(std::uint64_t from, std::uint64_t to) -> bool {
    return !validator::check_connection_hierarchy(model_.element_registry, from, to).has_value();
  }
  auto trigger_valid(std::uint64_t reaction, std::uint64_t trigger) -> bool {
    return !validator::check_trigger_hierarchy(model_.element_registry, reaction, trigger).has_value();
  }
  auto effect_valid(std::uint64_t reaction, std::uint64_t effect) -> bool {
    return !validator::check_effect_hierarchy(model_.element_registry, reaction, effect).has_value();
  }

  core::ReactorModel model_{};
};

TEST_F(HierarchyCheck, ConnectionAllowsSiblingWiring) {
  auto parent = add_reactor("parent");
  auto source = add_reactor("source", parent);
  auto sink = add_reactor("sink", parent);
  EXPECT_TRUE(connection_valid(add_output(source), add_input(sink)));
}

TEST_F(HierarchyCheck, ConnectionAllowsSiblingWiringAtTopLevel) {
  auto source = add_reactor("source");
  auto sink = add_reactor("sink");
  EXPECT_TRUE(connection_valid(add_output(source), add_input(sink)));
}

TEST_F(HierarchyCheck, ConnectionAllowsSelfLoop) {
  auto reactor = add_reactor("loop");
  EXPECT_TRUE(connection_valid(add_output(reactor), add_input(reactor)));
}

TEST_F(HierarchyCheck, ConnectionAllowsInputToChildInput) {
  auto parent = add_reactor("parent");
  auto child = add_reactor("child", parent);
  EXPECT_TRUE(connection_valid(add_input(parent), add_input(child)));
}

TEST_F(HierarchyCheck, ConnectionAllowsChildOutputToOutput) {
  auto parent = add_reactor("parent");
  auto child = add_reactor("child", parent);
  EXPECT_TRUE(connection_valid(add_output(child), add_output(parent)));
}

TEST_F(HierarchyCheck, ConnectionAllowsPassThrough) {
  auto reactor = add_reactor("relay");
  EXPECT_TRUE(connection_valid(add_input(reactor), add_output(reactor)));
}

TEST_F(HierarchyCheck, ConnectionRejectsInputToGrandchildInput) {
  auto parent = add_reactor("parent");
  auto child = add_reactor("child", parent);
  auto grandchild = add_reactor("grandchild", child);
  EXPECT_FALSE(connection_valid(add_input(parent), add_input(grandchild)));
}

TEST_F(HierarchyCheck, ConnectionRejectsGrandchildOutputToOutput) {
  auto parent = add_reactor("parent");
  auto child = add_reactor("child", parent);
  auto grandchild = add_reactor("grandchild", child);
  EXPECT_FALSE(connection_valid(add_output(grandchild), add_output(parent)));
}

TEST_F(HierarchyCheck, ConnectionRejectsWiringAcrossLevels) {
  auto parent = add_reactor("parent");
  auto sibling = add_reactor("sibling", parent);
  auto child = add_reactor("child", parent);
  auto grandchild = add_reactor("grandchild", child);
  EXPECT_FALSE(connection_valid(add_output(sibling), add_input(grandchild)));
}

TEST_F(HierarchyCheck, ConnectionRejectsWiringAcrossCousins) {
  auto left = add_reactor("left");
  auto right = add_reactor("right");
  auto left_child = add_reactor("child", left);
  auto right_child = add_reactor("child", right);
  EXPECT_FALSE(connection_valid(add_output(left_child), add_input(right_child)));
}

TEST_F(HierarchyCheck, ConnectionRejectsOutputIntoChildInput) {
  auto parent = add_reactor("parent");
  auto child = add_reactor("child", parent);
  EXPECT_FALSE(connection_valid(add_output(parent), add_input(child)));
}

TEST_F(HierarchyCheck, ConnectionRejectsChildOutputToInput) {
  auto parent = add_reactor("parent");
  auto child = add_reactor("child", parent);
  EXPECT_FALSE(connection_valid(add_output(child), add_input(parent)));
}

TEST_F(HierarchyCheck, ConnectionRejectsInputToSiblingInput) {
  auto parent = add_reactor("parent");
  auto left = add_reactor("left", parent);
  auto right = add_reactor("right", parent);
  EXPECT_FALSE(connection_valid(add_input(left), add_input(right)));
}

TEST_F(HierarchyCheck, ConnectionRejectsOutputToSiblingOutput) {
  auto parent = add_reactor("parent");
  auto left = add_reactor("left", parent);
  auto right = add_reactor("right", parent);
  EXPECT_FALSE(connection_valid(add_output(left), add_output(right)));
}

TEST_F(HierarchyCheck, ConnectionRejectsPassThroughAcrossReactors) {
  auto left = add_reactor("left");
  auto right = add_reactor("right");
  EXPECT_FALSE(connection_valid(add_input(left), add_output(right)));
}

TEST_F(HierarchyCheck, ConnectionErrorStatesTopLevelRule) {
  auto left = add_reactor("left");
  auto right = add_reactor("right");
  auto error = validator::check_connection_hierarchy(model_.element_registry, add_input(left), add_input(right));
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("top-level reactor's output port"), std::string::npos);
}

TEST_F(HierarchyCheck, ConnectionErrorStatesNestedRule) {
  auto parent = add_reactor("parent");
  auto left = add_reactor("left", parent);
  auto right = add_reactor("right", parent);
  auto error = validator::check_connection_hierarchy(model_.element_registry, add_input(left), add_input(right));
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("stays within one reactor"), std::string::npos);
}

TEST_F(HierarchyCheck, ConnectionRejectsNonPortEndpoints) {
  auto reactor = add_reactor("reactor");
  auto timer = add("timer", core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>()}, reactor);
  auto input = add_input(reactor);
  auto output = add_output(reactor);
  EXPECT_FALSE(connection_valid(timer, input));
  EXPECT_FALSE(connection_valid(output, timer));
  auto error = validator::check_connection_hierarchy(model_.element_registry, timer, input);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("periodic timer"), std::string::npos);
}

TEST_F(HierarchyCheck, TriggerAllowsOwnElements) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  EXPECT_TRUE(trigger_valid(reaction, add_input(reactor)));
  EXPECT_TRUE(
      trigger_valid(reaction, add("periodic_timer",
                                  core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>()}, reactor)));
  EXPECT_TRUE(trigger_valid(reaction, add("programmable_timer", core::ProgrammableTimerTag{}, reactor)));
  EXPECT_TRUE(trigger_valid(reaction, add("physical_event", core::PhysicalEventTag{}, reactor)));
  EXPECT_TRUE(trigger_valid(reaction, add("startup", core::StartupTag{}, reactor)));
  EXPECT_TRUE(trigger_valid(reaction, add("shutdown", core::ShutdownTag{}, reactor)));
}

TEST_F(HierarchyCheck, TriggerAllowsChildOutput) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  EXPECT_TRUE(trigger_valid(reaction, add_output(child)));
}

TEST_F(HierarchyCheck, TriggerRejectsOwnOutput) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  EXPECT_FALSE(trigger_valid(reaction, add_output(reactor)));
}

TEST_F(HierarchyCheck, TriggerRejectsChildElementsOtherThanOutputs) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  EXPECT_FALSE(trigger_valid(reaction, add_input(child)));
  EXPECT_FALSE(trigger_valid(reaction, add("timer", core::ProgrammableTimerTag{}, child)));
  EXPECT_FALSE(trigger_valid(reaction, add("startup", core::StartupTag{}, child)));
}

TEST_F(HierarchyCheck, TriggerRejectsGrandchildOutput) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  auto grandchild = add_reactor("grandchild", child);
  EXPECT_FALSE(trigger_valid(reaction, add_output(grandchild)));
}

TEST_F(HierarchyCheck, TriggerRejectsForeignElements) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto other = add_reactor("other");
  EXPECT_FALSE(trigger_valid(reaction, add_input(other)));
  EXPECT_FALSE(trigger_valid(reaction, add_output(other)));
}

TEST_F(HierarchyCheck, TriggerRejectsNonTriggerKinds) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto metric = add("metric", core::MetricTag{std::make_unique<core::MetricProperties>()}, reactor);
  EXPECT_FALSE(trigger_valid(reaction, metric));
}

TEST_F(HierarchyCheck, EffectAllowsOwnElements) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  EXPECT_TRUE(effect_valid(reaction, add_output(reactor)));
  EXPECT_TRUE(effect_valid(reaction, add("programmable_timer", core::ProgrammableTimerTag{}, reactor)));
  EXPECT_TRUE(effect_valid(reaction, add("shutdown", core::ShutdownTag{}, reactor)));
}

TEST_F(HierarchyCheck, EffectAllowsChildInput) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  EXPECT_TRUE(effect_valid(reaction, add_input(child)));
}

TEST_F(HierarchyCheck, EffectRejectsOwnInput) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  EXPECT_FALSE(effect_valid(reaction, add_input(reactor)));
}

TEST_F(HierarchyCheck, EffectRejectsOwnNonEffectKinds) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  EXPECT_FALSE(
      effect_valid(reaction, add("periodic_timer",
                                 core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>()}, reactor)));
  EXPECT_FALSE(effect_valid(reaction, add("startup", core::StartupTag{}, reactor)));
}

TEST_F(HierarchyCheck, EffectRejectsChildElementsOtherThanInputs) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  EXPECT_FALSE(effect_valid(reaction, add_output(child)));
  EXPECT_FALSE(effect_valid(reaction, add("timer", core::ProgrammableTimerTag{}, child)));
}

TEST_F(HierarchyCheck, EffectRejectsGrandchildInput) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  auto grandchild = add_reactor("grandchild", child);
  EXPECT_FALSE(effect_valid(reaction, add_input(grandchild)));
}

TEST_F(HierarchyCheck, ModelCheckReportsEveryViolation) {
  auto reactor = add_reactor("reactor");
  auto reaction = add_reaction(reactor);
  auto child = add_reactor("child", reactor);
  auto grandchild = add_reactor("grandchild", child);
  auto grandchild_input = add_input(grandchild);
  auto grandchild_output = add_output(grandchild);
  auto reactor_input = add_input(reactor);

  model_.reaction_dependency_registry.register_reaction_trigger(reaction, grandchild_output);
  model_.reaction_dependency_registry.register_reaction_effect(reaction, grandchild_input);
  ASSERT_TRUE(model_.connection_graph.add_connection(
      {.from_uid = reactor_input, .to_uid = grandchild_input, .delay = std::nullopt}));

  auto result = validator::check_hierarchy(model_);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 3);
  EXPECT_NE(result.error()[0].find("Cannot connect port reactor.input to port reactor.child.grandchild.input"),
            std::string::npos);
  EXPECT_NE(result.error()[1].find("may not use reactor.child.grandchild.output as a trigger"), std::string::npos);
  EXPECT_NE(result.error()[2].find("may not use reactor.child.grandchild.input as an effect"), std::string::npos);
}

// The checks behind the hierarchy gate assume connection endpoints are
// ports; run on this model, check_cross_boundary_serializers would throw
// instead of reporting. The gate turns that into a plain hierarchy error.
TEST_F(HierarchyCheck, HierarchyFailureGatesLaterChecks) {
  auto reactor = add_reactor("reactor");
  auto timer = add("timer", core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>()}, reactor);
  auto input = add_input(reactor);
  ASSERT_TRUE(model_.connection_graph.add_connection(
      {.from_uid = timer, .to_uid = input, .delay = std::nullopt, .crossing = core::BoundaryCrossing::Both}));

  auto result = validator::run_all_checks(model_);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1);
  EXPECT_NE(result.error().front().find("connections link ports"), std::string::npos);
}

TEST_F(HierarchyCheck, ModelCheckAcceptsConformingModel) {
  auto parent = add_reactor("parent");
  auto reaction = add_reaction(parent);
  auto child = add_reactor("child", parent);
  auto child_input = add_input(child);
  auto child_output = add_output(child);
  auto parent_input = add_input(parent);
  auto parent_output = add_output(parent);

  model_.reaction_dependency_registry.register_reaction_trigger(reaction, parent_input);
  model_.reaction_dependency_registry.register_reaction_trigger(reaction, child_output);
  model_.reaction_dependency_registry.register_reaction_effect(reaction, child_input);
  ASSERT_TRUE(model_.connection_graph.add_connection(
      {.from_uid = child_output, .to_uid = parent_output, .delay = std::nullopt}));

  EXPECT_TRUE(validator::check_hierarchy(model_).has_value());
}

} // namespace
