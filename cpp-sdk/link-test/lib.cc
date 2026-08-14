// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// One of the translation units of the SDK link test (see CMakeLists.txt).
// This one is compiled into a shared library -- the link test's second DSO,
// with the shape an installed node .so has: it registers a node with
// XRONOS_REGISTER_NODE, so the shared library is built with -Wl,--no-undefined
// (see CMakeLists.txt) and the whole node entry-point path -- the borrowing
// program context, the context construction, the node's reactor tree -- must
// resolve to header-inline code or an allowlisted compiled symbol, or the link
// fails. The executable that links this library also runs its reactor program
// (run_lib_program) to check that two copies of the inline SDK coexist in one
// process.

#include "include_all.hh"

namespace xronos::sdk::link_test {

namespace {

constexpr int magic{42};

class Sender final : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

private:
  OutputPort<int> output_{"output", context()};

  class Send : public Reaction<Sender> {
    using Reaction<Sender>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final { output_effect.set(magic); }
  };

  void assemble() final { add_reaction<Send>("send"); }
};

class Receiver final : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }
  [[nodiscard]] auto received() const noexcept -> bool { return received_; }

private:
  InputPort<int> input_{"input", context()};

  bool received_{false};

  class Receive : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<int> input_trigger{self().input_, context()};
    void handler() final {
      const auto value = input_trigger.get();
      self().received_ = value != nullptr && *value == magic;
    }
  };

  void assemble() final { add_reaction<Receive>("receive"); }
};

} // namespace

// A node with the same sender/receiver program, registered so the shared
// library carries the xronos run entry points and links them under
// --no-undefined. It is not executed here (that needs a host-provided backend);
// registering it is enough to force the entry-point path through the linker.
// Unlike the reactors above, it lives in the named namespace rather than the
// anonymous one so the global-scope XRONOS_REGISTER_NODE below can name it by
// its qualified name.
class LinkTestNode final : public Node {
public:
  using Node::Node;

private:
  Sender sender_{"sender", context()};
  Receiver receiver_{"receiver", context()};

  void assemble() final { connect(sender_.output(), receiver_.input()); }
};

auto run_lib_program() -> bool {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input());
  env.execute();
  return receiver.received();
}

} // namespace xronos::sdk::link_test

XRONOS_REGISTER_NODE(xronos::sdk::link_test::LinkTestNode, "link_test_node")
