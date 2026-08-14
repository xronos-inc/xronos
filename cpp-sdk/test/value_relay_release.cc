// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// This translation unit defines NDEBUG, independent of the build type. It
// pins down semantics that only exist when assertions are compiled out:
// precondition violations in the relay overloads must degrade to no-ops.
// The overloads are header templates, so defining NDEBUG here compiles the
// release-mode code paths even in a debug test build. The define must stay
// above all includes.
#define NDEBUG // NOLINT(cppcoreguidelines-macro-usage)

#include <string>

#include "xronos/sdk.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/reactor.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test::value_relay_release {

struct Message {
  std::string text;
};

// Relays views of a trigger that never carries an event. Such views are
// absent but still refer to the runtime's (empty) stored value, so this
// pins down that the relay overloads guard on the view's presence and not
// on the underlying value pointer: nothing may be sent or scheduled.
class AbsentRelay : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void check_post_conditions() const {
    EXPECT_EQ(relays_attempted_, 1U);
    EXPECT_EQ(port_events_received_, 0U);
    EXPECT_EQ(timer_events_received_, 0U);
  }

private:
  ProgrammableTimer<Message> timer_{"timer", context()};
  InputPort<Message> never_set_{"never_set", context()};
  InputPort<Message> input_{"input", context()};
  OutputPort<Message> output_{"output", context()};

  unsigned relays_attempted_{0};
  unsigned port_events_received_{0};
  unsigned timer_events_received_{0};

  class OnStartup : public Reaction<AbsentRelay> {
    using Reaction<AbsentRelay>::Reaction;
    Trigger<void> startup_trigger_{self().startup(), context()};
    Trigger<Message> never_set_trigger_{self().never_set_, context()};
    PortEffect<Message> output_effect_{self().output_, context()};
    ProgrammableTimerEffect<Message> timer_effect_{self().timer_, context()};
    void handler() final {
      self().relays_attempted_++;
      ASSERT_FALSE(never_set_trigger_.get());
      output_effect_.set(never_set_trigger_.get());
      timer_effect_.schedule(never_set_trigger_.get(), 1ms);
    }
  };

  class OnInput : public Reaction<AbsentRelay> {
    using Reaction<AbsentRelay>::Reaction;
    Trigger<Message> input_trigger_{self().input_, context()};
    void handler() final { self().port_events_received_++; }
  };

  class OnTimer : public Reaction<AbsentRelay> {
    using Reaction<AbsentRelay>::Reaction;
    Trigger<Message> timer_trigger_{self().timer_, context()};
    void handler() final { self().timer_events_received_++; }
  };

  void assemble() final {
    add_reaction<OnStartup>("on_startup");
    add_reaction<OnInput>("on_input");
    add_reaction<OnTimer>("on_timer");
  }
};

TEST(value_relay_release, relaying_an_absent_view_sends_nothing) {
  TestEnvironment env{};
  AbsentRelay relay{"relay", env.context()};
  env.connect(relay.output(), relay.input());
  env.execute();
  relay.check_post_conditions();
}

} // namespace xronos::sdk::test::value_relay_release
