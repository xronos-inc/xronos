// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// Exercises detail::create_node, the function behind XRONOS_REGISTER_NODE's
// creator entry points: a host constructs a registered node against its own
// backend, either at the top level or as a child of a reactor the host
// registered itself. These tests stand in for that host, reaching the
// environment's backend the way a host reaches its own, and hold the created
// node through the opaque abi::Node handle a host holds.

#include <memory>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

namespace xronos::sdk::test {

namespace {

// Set by ChildNode's startup reaction; reset by each test that runs one.
// Handlers may run on runtime worker threads, where gtest assertions are not
// safe, so the handler only records and the test checks after execute().
bool child_reaction_ran{false};

// A minimal registered-node stand-in: one startup reaction that records its
// invocation, so a test can prove the node participated in the host's run.
class ChildNode final : public Node {
public:
  using Node::Node;

private:
  class OnStartup : public Reaction<ChildNode> {
    using Reaction<ChildNode>::Reaction;
    Trigger<void> startup_trigger_{self().startup(), context()};
    void handler() final { child_reaction_ran = true; }
  };

  void assemble() final { add_reaction<OnStartup>("on_startup"); }
};

// The host-registered reactor a node is created under.
class Host final : public Reactor {
public:
  using Reactor::Reactor;

private:
  void assemble() final {}
};

} // namespace

TEST(nodes, ANodeCreatedWithAParentBecomesThatReactorsChild) {
  child_reaction_ran = false;
  TestEnvironment env{};
  auto environment_context = env.context();
  auto& backend = detail::ContextAccess::get_program_context(environment_context)->backend();

  Host host{"host", env.context()};
  const std::unique_ptr<abi::Node> created{detail::create_node<ChildNode>("child", &backend, host.uid())};

  // The host only holds the opaque handle; the concrete type is known here
  // because the test defined it, so it can read the element surface back.
  const auto* child = dynamic_cast<const ChildNode*>(created.get());
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->fqn(), "host.child");

  // The nested node is a full member of the host's program: its assemble()
  // runs with everyone else's and its reaction fires.
  env.execute();
  EXPECT_TRUE(child_reaction_ran);
}

TEST(nodes, ANodeCreatedWithoutAParentIsTopLevel) {
  child_reaction_ran = false;
  TestEnvironment env{};
  auto environment_context = env.context();
  auto& backend = detail::ContextAccess::get_program_context(environment_context)->backend();

  const std::unique_ptr<abi::Node> created{detail::create_node<ChildNode>("solo", &backend)};

  const auto* node = dynamic_cast<const ChildNode*>(created.get());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->fqn(), "solo");

  env.execute();
  EXPECT_TRUE(child_reaction_ran);
}

} // namespace xronos::sdk::test
