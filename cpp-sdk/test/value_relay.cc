// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <string>

#include "xronos/sdk.hh"
#include "xronos/sdk/physical_event.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/reactor.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test::value_relay {

struct Message {
  std::string text;
};

} // namespace xronos::sdk::test::value_relay

// Instantiates all (non-template) members of PhysicalEvent, including the
// Value and ValueView trigger overloads that the runtime test below cannot
// reach from a reaction handler. (Explicit instantiation must appear in an
// enclosing namespace of the template.)
template class xronos::sdk::PhysicalEvent<xronos::sdk::test::value_relay::Message>;

namespace xronos::sdk::test::value_relay {

// Passes a value through every relay overload: schedule(ValueView),
// schedule(Value), set(ValueView), and set(Value).
class Relay : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void check_post_conditions() const {
    EXPECT_EQ(views_received_, 1U);
    EXPECT_EQ(values_received_, 1U);
  }

private:
  ProgrammableTimer<Message> timer_{"timer", context()};
  InputPort<Message> input_{"input", context()};
  OutputPort<Message> output_{"output", context()};

  unsigned timer_hops_{0};
  unsigned views_received_{0};
  unsigned values_received_{0};

  class OnStartup : public Reaction<Relay> {
    using Reaction<Relay>::Reaction;
    Trigger<void> startup_trigger_{self().startup(), context()};
    ProgrammableTimerEffect<Message> timer_effect_{self().timer_, context()};
    void handler() final { timer_effect_.schedule(Value<Message>{Message{"relayed"}}); }
  };

  class OnTimer : public Reaction<Relay> {
    using Reaction<Relay>::Reaction;
    Trigger<Message> timer_trigger_{self().timer_, context()};
    ProgrammableTimerEffect<Message> timer_effect_{self().timer_, context()};
    PortEffect<Message> output_effect_{self().output_, context()};
    void handler() final {
      auto view = timer_trigger_.get();
      ASSERT_TRUE(view);
      EXPECT_EQ(view->text, "relayed");
      if (self().timer_hops_++ == 0) {
        // Relay the borrowed view into a future event.
        timer_effect_.schedule(view, 1ms);
      } else {
        // Relay the view to the port, then overwrite with an owned value; the
        // last write wins, so the receiver sees the owned value.
        output_effect_.set(view);
        output_effect_.set(Value<Message>{Message{"owned"}});
      }
    }
  };

  class OnInput : public Reaction<Relay> {
    using Reaction<Relay>::Reaction;
    Trigger<Message> input_trigger_{self().input_, context()};
    void handler() final {
      if (auto view = input_trigger_.get()) {
        self().views_received_++;
        if (view->text == "owned") {
          self().values_received_++;
        }
      }
    }
  };

  void assemble() final {
    add_reaction<OnStartup>("on_startup");
    add_reaction<OnTimer>("on_timer");
    add_reaction<OnInput>("on_input");
  }
};

TEST(value_relay, views_and_values_relay_through_effects) {
  TestEnvironment env{};
  Relay relay{"relay", env.context()};
  env.connect(relay.output(), relay.input());
  env.execute();
  relay.check_post_conditions();
}

} // namespace xronos::sdk::test::value_relay
