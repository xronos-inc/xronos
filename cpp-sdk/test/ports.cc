// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reaction.hh"

#include "gtest/gtest.h"
#include <string>
#include <string_view>

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

namespace simple_void {

class Sender : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

private:
  OutputPort<void> output_{"output", context()};

  class Send : public Reaction<Sender> {
    using Reaction<Sender>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<void> output_effect{self().output_, context()};
    void handler() final { output_effect.set(); }
  };

  void assemble() final { add_reaction<Send>("send"); }
};

class Receiver : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }
  [[nodiscard]] auto input() noexcept -> auto& { return input_; }

  void check_post_conditions(Duration expected_time) {
    EXPECT_TRUE(reaction_executed_);
    EXPECT_EQ(reaction_executed_at_, expected_time);
  }

private:
  InputPort<void> input_{"input", context()};

  bool reaction_executed_{false};
  Duration reaction_executed_at_{Duration::min()};

  class Send : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<void> input_trigger{self().input_, context()};
    void handler() final {
      EXPECT_TRUE(input_trigger.is_present());
      EXPECT_FALSE(self().reaction_executed_);
      self().reaction_executed_at_ = elapsed_time();
      self().reaction_executed_ = true;
    }
  };
  void assemble() final { add_reaction<Send>("send"); }
};

TEST(ports, SimpleVoidNoDelay) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input());
  env.execute();
  receiver.check_post_conditions(0s);
}

TEST(ports, SimpleVoid1sDelay) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input(), 1s);
  env.execute();
  receiver.check_post_conditions(1s);
}

TEST(ports, SimpleVoidNoDelay2Receivers) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context()};
  Receiver receiver2{"receiver2", env.context()};
  env.connect(sender.output(), receiver1.input());
  env.connect(sender.output(), receiver2.input());
  env.execute();
  receiver1.check_post_conditions(0s);
  receiver2.check_post_conditions(0s);
}

TEST(ports, SimpleVoid1sDelay2Receivers) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context()};
  Receiver receiver2{"receiver2", env.context()};
  env.connect(sender.output(), receiver1.input(), 1s);
  env.connect(sender.output(), receiver2.input(), 1s);
  env.execute();
  receiver1.check_post_conditions(1s);
  receiver2.check_post_conditions(1s);
}

TEST(ports, SimpleVoidMixedDelay2Receivers) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context()};
  Receiver receiver2{"receiver2", env.context()};
  env.connect(sender.output(), receiver1.input(), 1s);
  env.connect(sender.output(), receiver2.input());
  env.execute();
  receiver1.check_post_conditions(1s);
  receiver2.check_post_conditions(0s);
}

} // namespace simple_void

namespace simple_int {

constexpr static int value{42};

class Sender : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

private:
  OutputPort<int> output_{"output", context()};

  class Receive : public Reaction<Sender> {
    using Reaction<Sender>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final { output_effect.set(value); }
  };

  void assemble() final { add_reaction<Receive>("receive"); }
};

class Receiver : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }
  [[nodiscard]] auto input() noexcept -> auto& { return input_; }

  void check_post_conditions(Duration expected_time) {
    EXPECT_TRUE(reaction_executed_);
    EXPECT_EQ(reaction_executed_at_, expected_time);
  }

private:
  InputPort<int> input_{"input", context()};

  bool reaction_executed_{false};
  Duration reaction_executed_at_{Duration::min()};

  class Receive : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<int> input_trigger{self().input_, context()};
    void handler() final {
      EXPECT_TRUE(input_trigger.is_present());
      EXPECT_FALSE(self().reaction_executed_);
      EXPECT_NE(input_trigger.get(), nullptr);
      EXPECT_EQ(*input_trigger.get(), value);
      self().reaction_executed_at_ = elapsed_time();
      self().reaction_executed_ = true;
    }
  };
  void assemble() final { add_reaction<Receive>("receive"); }
};

TEST(ports, SimpleIntNoDelay) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input());
  env.execute();
  receiver.check_post_conditions(0s);
}

TEST(ports, SimpleInt1sDelay) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input(), 1s);
  env.execute();
  receiver.check_post_conditions(1s);
}

TEST(ports, SimpleIntNoDelay2Receivers) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context()};
  Receiver receiver2{"receiver2", env.context()};
  env.connect(sender.output(), receiver1.input());
  env.connect(sender.output(), receiver2.input());
  env.execute();
  receiver1.check_post_conditions(0s);
  receiver2.check_post_conditions(0s);
}

TEST(ports, SimpleInt1sDelay2Receivers) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context()};
  Receiver receiver2{"receiver2", env.context()};
  env.connect(sender.output(), receiver1.input(), 1s);
  env.connect(sender.output(), receiver2.input(), 1s);
  env.execute();
  receiver1.check_post_conditions(1s);
  receiver2.check_post_conditions(1s);
}

TEST(ports, SimpleIntMixedDelay2Receivers) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context()};
  Receiver receiver2{"receiver2", env.context()};
  env.connect(sender.output(), receiver1.input(), 1s);
  env.connect(sender.output(), receiver2.input());
  env.execute();
  receiver1.check_post_conditions(1s);
  receiver2.check_post_conditions(0s);
}

} // namespace simple_int

namespace effect_read_back {

constexpr static int value{simple_int::value};

// A reaction observes its own write through the effect: before set() the
// effect reads as absent, and after set() is_present() returns true and
// get() yields the written value.
class Sender : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

  void check_post_conditions() const { EXPECT_TRUE(reaction_executed_); }

private:
  OutputPort<int> output_{"output", context()};

  bool reaction_executed_{false};

  class Send : public Reaction<Sender> {
    using Reaction<Sender>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final {
      EXPECT_FALSE(output_effect.is_present());
      EXPECT_EQ(output_effect.get(), nullptr);
      output_effect.set(value);
      EXPECT_TRUE(output_effect.is_present());
      ASSERT_NE(output_effect.get(), nullptr);
      EXPECT_EQ(*output_effect.get(), value);
      self().reaction_executed_ = true;
    }
  };

  void assemble() final { add_reaction<Send>("send"); }
};

TEST(ports, EffectReadsBackTheSetValue) {
  TestEnvironment env{};
  Sender sender{"sender", env.context()};
  simple_int::Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input());
  env.execute();
  sender.check_post_conditions();
  receiver.check_post_conditions(0s);
}

} // namespace effect_read_back

// TEST_CASE("Sending repeatedly", "[ports]") {

namespace sending_repeatedly {

class Sender : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

private:
  PeriodicTimer timer_{"timer", context(), 1s};
  OutputPort<int> output_{"output", context()};
  int counter_{0};

  class Send : public Reaction<Sender> {
    using Reaction<Sender>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final { output_effect.set(self().counter_++); }
  };

  void assemble() final { add_reaction<Send>("send"); }
};

class Receiver : public Reactor {
public:
  Receiver(std::string_view name, Context parent_context, Duration delay)
      : Reactor(name, parent_context)
      , timer_expected_{"timer_expected", context(), 1s, delay}
      , timer_offset_{"timer_offset", context(), 1s, 500ms + delay} {}
  using Reactor::Reactor;

  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void check_post_conditions(int expected_count) const {
    EXPECT_EQ(counter_, expected_count);
    EXPECT_TRUE(check_timing_executed_);
  }

private:
  PeriodicTimer timer_expected_{"timer_expected", context(), 1s};
  PeriodicTimer timer_offset_{"timer_offset", context(), 1s, 500ms};
  InputPort<int> input_{"input", context()};

  int counter_{0};
  bool check_timing_executed_{false};

  class Receive : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<int> input_trigger{self().input_, context()};
    void handler() final {
      EXPECT_TRUE(input_trigger.is_present());
      EXPECT_NE(input_trigger.get(), nullptr);
      EXPECT_EQ(*input_trigger.get(), self().counter_);
      self().counter_++;
    }
  };

  class CheckTiming : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<void> expected_trigger{self().timer_expected_, context()};
    Trigger<void> offset_trigger{self().timer_offset_, context()};
    Trigger<int> input_trigger{self().input_, context()};
    void handler() final {
      EXPECT_TRUE(expected_trigger.is_present() || offset_trigger.is_present());
      EXPECT_FALSE(expected_trigger.is_present() && offset_trigger.is_present());
      if (expected_trigger.is_present()) {
        EXPECT_TRUE(input_trigger.is_present());
      } else {
        EXPECT_FALSE(input_trigger.is_present());
      }
      self().check_timing_executed_ = true;
    }
  };

  void assemble() final {
    add_reaction<Receive>("receive");
    add_reaction<CheckTiming>("check_timing");
  }
};

TEST(ports, SendingRepeatedlyNoDelay) {
  TestEnvironment env{5s};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context(), 0s};
  env.connect(sender.output(), receiver.input());
  env.execute();
  receiver.check_post_conditions(6);
}

TEST(ports, SendingRepeatedly1sDelay) {
  TestEnvironment env{5s};
  Sender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context(), 1s};
  env.connect(sender.output(), receiver.input(), 1s);
  env.execute();
  receiver.check_post_conditions(5);
}

TEST(ports, SendingRepeatedlyNoDelay2Receivers) {
  TestEnvironment env{5s};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context(), 0s};
  Receiver receiver2{"receiver2", env.context(), 0s};
  env.connect(sender.output(), receiver1.input());
  env.connect(sender.output(), receiver2.input());
  env.execute();
  receiver1.check_post_conditions(6);
  receiver2.check_post_conditions(6);
}

TEST(ports, SendingRepeatedly1sDelay2Receivers) {
  TestEnvironment env{5s};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context(), 1s};
  Receiver receiver2{"receiver2", env.context(), 1s};
  env.connect(sender.output(), receiver1.input(), 1s);
  env.connect(sender.output(), receiver2.input(), 1s);
  env.execute();
  receiver1.check_post_conditions(5);
  receiver2.check_post_conditions(5);
}

TEST(ports, SendingRepeatedlyMixedDelay2Receivers) {
  TestEnvironment env{5s};
  Sender sender{"sender", env.context()};
  Receiver receiver1{"receiver1", env.context(), 1s};
  Receiver receiver2{"receiver2", env.context(), 0s};
  env.connect(sender.output(), receiver1.input(), 1s);
  env.connect(sender.output(), receiver2.input());
  env.execute();
  receiver1.check_post_conditions(5);
  receiver2.check_post_conditions(6);
}

} // namespace sending_repeatedly

namespace nested_connection {

class SenderWrapper : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

private:
  simple_int::Sender sender_{"sender", context()};
  OutputPort<int> output_{"output", context()};

  void assemble() final { connect(sender_.output(), output_); }
};

class ReceiverWrapper : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }
  [[nodiscard]] auto input() noexcept -> auto& { return input_; }

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_int::Receiver receiver_{"receiver", context()};
  InputPort<int> input_{"input", context()};

  void assemble() final { connect(input_, receiver_.input()); }
};

TEST(ports, NestedNoDelay) {
  TestEnvironment env{};
  SenderWrapper sender{"sender", env.context()};
  ReceiverWrapper receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input());
  env.execute();
  receiver.check_post_conditions(0s);
}

TEST(ports, Nested1sDelay) {
  TestEnvironment env{};
  SenderWrapper sender{"sender", env.context()};
  ReceiverWrapper receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input(), 1s);
  env.execute();
  receiver.check_post_conditions(1s);
}

} // namespace nested_connection

namespace nested_reaction {

class WrapperVoid : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_void::Sender sender_{"sender", context()};
  simple_void::Receiver receiver_{"receiver", context()};

  class ForwardReaction : public Reaction<WrapperVoid> {
  public:
    using Reaction<WrapperVoid>::Reaction;
    Trigger<void> trigger{self().sender_.output(), context()};
    PortEffect<void> effect{self().receiver_.input(), context()};
    void handler() final { effect.set(); }
  };

  void assemble() final { add_reaction<ForwardReaction>("forward"); }
};

class WrapperInt : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_int::Sender sender_{"sender", context()};
  simple_int::Receiver receiver_{"receiver", context()};

  class ForwardReaction : public Reaction<WrapperInt> {
  public:
    using Reaction<WrapperInt>::Reaction;
    Trigger<int> trigger{self().sender_.output(), context()};
    PortEffect<int> effect{self().receiver_.input(), context()};
    void handler() final { effect.set(trigger.get()); }
  };

  void assemble() final { add_reaction<ForwardReaction>("forward"); }
};

class WrapperIntDelayed : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_int::Sender sender_{"sender", context()};
  simple_int::Receiver receiver_{"receiver", context()};
  ProgrammableTimer<int> timer_{"timer", context()};

  class ReceiveReaction : public Reaction<WrapperIntDelayed> {
  public:
    using Reaction<WrapperIntDelayed>::Reaction;
    Trigger<int> trigger{self().sender_.output(), context()};
    ProgrammableTimerEffect<int> effect{self().timer_, context()};
    void handler() final { effect.schedule(trigger.get(), 1s); }
  };

  class ForwardReaction : public Reaction<WrapperIntDelayed> {
  public:
    using Reaction<WrapperIntDelayed>::Reaction;
    Trigger<int> trigger{self().timer_, context()};
    PortEffect<int> effect{self().receiver_.input(), context()};
    void handler() final { effect.set(trigger.get()); }
  };

  void assemble() final {
    add_reaction<ReceiveReaction>("receive");
    add_reaction<ForwardReaction>("forward");
  }
};

// Covers only the trigger direction: a reaction triggered by a contained
// reactor's output port, with no effect on any nested port.
class ObserverWrapper : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) {
    EXPECT_TRUE(reaction_executed_);
    EXPECT_EQ(reaction_executed_at_, expected_time);
  }

private:
  simple_int::Sender sender_{"sender", context()};

  bool reaction_executed_{false};
  Duration reaction_executed_at_{Duration::min()};

  class ObserveReaction : public Reaction<ObserverWrapper> {
  public:
    using Reaction<ObserverWrapper>::Reaction;
    Trigger<int> trigger{self().sender_.output(), context()};
    void handler() final {
      EXPECT_TRUE(trigger.is_present());
      EXPECT_FALSE(self().reaction_executed_);
      EXPECT_NE(trigger.get(), nullptr);
      EXPECT_EQ(*trigger.get(), simple_int::value);
      self().reaction_executed_at_ = elapsed_time();
      self().reaction_executed_ = true;
    }
  };

  void assemble() final { add_reaction<ObserveReaction>("observe"); }
};

// Covers only the effect direction: a reaction writing to a contained
// reactor's unconnected input port, with no trigger on any nested port.
class WriterWrapper : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_int::Receiver receiver_{"receiver", context()};

  class WriteReaction : public Reaction<WriterWrapper> {
  public:
    using Reaction<WriterWrapper>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> effect{self().receiver_.input(), context()};
    void handler() final { effect.set(simple_int::value); }
  };

  void assemble() final { add_reaction<WriteReaction>("write"); }
};

// Covers writing to a contained input port that has an outgoing connection:
// the written value continues through the wrapped receiver's pass-through
// connection.
class ConnectedInputWriter : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_wrapper_.check_post_conditions(expected_time); }

private:
  nested_connection::ReceiverWrapper receiver_wrapper_{"receiver_wrapper", context()};

  class WriteReaction : public Reaction<ConnectedInputWriter> {
  public:
    using Reaction<ConnectedInputWriter>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> effect{self().receiver_wrapper_.input(), context()};
    void handler() final { effect.set(simple_int::value); }
  };

  void assemble() final { add_reaction<WriteReaction>("write"); }
};

// Covers triggering on a contained output port that has an incoming
// connection: the wrapped sender's value arrives through the pass-through
// connection and triggers the wrapper's reaction.
class ConnectedOutputObserver : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) {
    EXPECT_TRUE(reaction_executed_);
    EXPECT_EQ(reaction_executed_at_, expected_time);
  }

private:
  nested_connection::SenderWrapper sender_wrapper_{"sender_wrapper", context()};

  bool reaction_executed_{false};
  Duration reaction_executed_at_{Duration::min()};

  class ObserveReaction : public Reaction<ConnectedOutputObserver> {
  public:
    using Reaction<ConnectedOutputObserver>::Reaction;
    Trigger<int> trigger{self().sender_wrapper_.output(), context()};
    void handler() final {
      EXPECT_TRUE(trigger.is_present());
      EXPECT_FALSE(self().reaction_executed_);
      EXPECT_NE(trigger.get(), nullptr);
      EXPECT_EQ(*trigger.get(), simple_int::value);
      self().reaction_executed_at_ = elapsed_time();
      self().reaction_executed_ = true;
    }
  };

  void assemble() final { add_reaction<ObserveReaction>("observe"); }
};

// Covers a contained output port that fans out: a connection feeds an inner
// receiver while the wrapper's own reaction observes the same port
// directly. Both deliveries arrive at the same tag.
class ObservedAndConnectedOutput : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) {
    EXPECT_TRUE(reaction_executed_);
    EXPECT_EQ(reaction_executed_at_, expected_time);
    receiver_.check_post_conditions(expected_time);
  }

private:
  simple_int::Sender sender_{"sender", context()};
  simple_int::Receiver receiver_{"receiver", context()};

  bool reaction_executed_{false};
  Duration reaction_executed_at_{Duration::min()};

  class ObserveReaction : public Reaction<ObservedAndConnectedOutput> {
  public:
    using Reaction<ObservedAndConnectedOutput>::Reaction;
    Trigger<int> trigger{self().sender_.output(), context()};
    void handler() final {
      EXPECT_TRUE(trigger.is_present());
      EXPECT_FALSE(self().reaction_executed_);
      EXPECT_NE(trigger.get(), nullptr);
      EXPECT_EQ(*trigger.get(), simple_int::value);
      self().reaction_executed_at_ = elapsed_time();
      self().reaction_executed_ = true;
    }
  };

  void assemble() final {
    connect(sender_.output(), receiver_.input());
    add_reaction<ObserveReaction>("observe");
  }
};

// Covers two reactions of one reactor writing the same contained input
// port. Both writes land at the same tag, so the later reaction's value is
// the one delivered.
class DoubleWriterWrapper : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_int::Receiver receiver_{"receiver", context()};

  class WriteFirst : public Reaction<DoubleWriterWrapper> {
  public:
    using Reaction<DoubleWriterWrapper>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> effect{self().receiver_.input(), context()};
    void handler() final { effect.set(simple_int::value - 1); }
  };

  class WriteSecond : public Reaction<DoubleWriterWrapper> {
  public:
    using Reaction<DoubleWriterWrapper>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> effect{self().receiver_.input(), context()};
    void handler() final { effect.set(simple_int::value); }
  };

  void assemble() final {
    add_reaction<WriteFirst>("write_first");
    add_reaction<WriteSecond>("write_second");
  }
};

// Covers two reactions of one reactor triggering on the same contained
// output port. Both run at the delivery tag, in reaction order.
class DoubleObserverWrapper : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) {
    EXPECT_EQ(first_executed_at_, expected_time);
    EXPECT_EQ(second_executed_at_, expected_time);
    EXPECT_TRUE(first_ran_before_second_);
  }

private:
  simple_int::Sender sender_{"sender", context()};

  Duration first_executed_at_{Duration::min()};
  Duration second_executed_at_{Duration::min()};
  bool first_ran_before_second_{false};

  class ObserveFirst : public Reaction<DoubleObserverWrapper> {
  public:
    using Reaction<DoubleObserverWrapper>::Reaction;
    Trigger<int> trigger{self().sender_.output(), context()};
    void handler() final {
      EXPECT_TRUE(trigger.is_present());
      EXPECT_NE(trigger.get(), nullptr);
      EXPECT_EQ(*trigger.get(), simple_int::value);
      self().first_executed_at_ = elapsed_time();
    }
  };

  class ObserveSecond : public Reaction<DoubleObserverWrapper> {
  public:
    using Reaction<DoubleObserverWrapper>::Reaction;
    Trigger<int> trigger{self().sender_.output(), context()};
    void handler() final {
      EXPECT_TRUE(trigger.is_present());
      EXPECT_NE(trigger.get(), nullptr);
      EXPECT_EQ(*trigger.get(), simple_int::value);
      self().first_ran_before_second_ = self().first_executed_at_ != Duration::min();
      self().second_executed_at_ = elapsed_time();
    }
  };

  void assemble() final {
    add_reaction<ObserveFirst>("observe_first");
    add_reaction<ObserveSecond>("observe_second");
  }
};

// Covers a nested write at the last tag before shutdown. Validation
// forbids effects on shutdown-triggered reactions, so the writing reaction
// requests shutdown at the tag it writes instead; that is the closest a
// write can land to the shutdown tag. The value must still arrive at that
// tag and the program must terminate.
class WriteThenShutdownWrapper : public Reactor {
public:
  using Reactor::Reactor;

  void check_post_conditions(Duration expected_time) { receiver_.check_post_conditions(expected_time); }

private:
  simple_int::Receiver receiver_{"receiver", context()};

  class WriteAndRequestShutdown : public Reaction<WriteThenShutdownWrapper> {
  public:
    using Reaction<WriteThenShutdownWrapper>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> effect{self().receiver_.input(), context()};
    ShutdownEffect shutdown_effect{self().shutdown(), context()};
    void handler() final {
      effect.set(simple_int::value);
      shutdown_effect.trigger_shutdown();
    }
  };

  void assemble() final { add_reaction<WriteAndRequestShutdown>("write_and_request_shutdown"); }
};

TEST(ports, NestedReactionVoid) {
  TestEnvironment env{};
  WrapperVoid wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionInt) {
  TestEnvironment env{};
  WrapperInt wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionTriggerOnly) {
  TestEnvironment env{};
  ObserverWrapper wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionEffectOnly) {
  TestEnvironment env{};
  WriterWrapper wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionWriteToConnectedInput) {
  TestEnvironment env{};
  ConnectedInputWriter wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionTriggerFromConnectedOutput) {
  TestEnvironment env{};
  ConnectedOutputObserver wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionIntDelayed) {
  TestEnvironment env{};
  WrapperIntDelayed wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(1s);
}

TEST(ports, NestedReactionTriggerBesideConnection) {
  TestEnvironment env{};
  ObservedAndConnectedOutput wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionTwoWriters) {
  TestEnvironment env{};
  DoubleWriterWrapper wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionTwoObservers) {
  TestEnvironment env{};
  DoubleObserverWrapper wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

TEST(ports, NestedReactionWriteBeforeShutdown) {
  TestEnvironment env{};
  WriteThenShutdownWrapper wrapper{"wrapper", env.context()};
  env.execute();
  wrapper.check_post_conditions(0s);
}

} // namespace nested_reaction

namespace never_written {

// A sender that declares an output port but has no reaction writing it.
class SilentSender : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }

private:
  OutputPort<void> output_{"output", context()};

  void assemble() final {}
};

class Receiver : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto input() noexcept -> auto& { return input_; }

  void check_post_conditions() const { EXPECT_FALSE(reaction_executed_); }

private:
  InputPort<void> input_{"input", context()};

  bool reaction_executed_{false};

  class Receive : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<void> input_trigger{self().input_, context()};
    void handler() final { self().reaction_executed_ = true; }
  };

  void assemble() final { add_reaction<Receive>("receive"); }
};

// A connected port that no reaction writes never delivers an event. The
// program must terminate without invoking the downstream handler.
TEST(ports, NeverWrittenConnectedPortTerminates) {
  TestEnvironment env{100ms};
  SilentSender sender{"sender", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(sender.output(), receiver.input());
  env.execute();
  receiver.check_post_conditions();
}

} // namespace never_written

namespace readback {

class InputReadback : public Reactor {
public:
  using Reactor::Reactor;

private:
  InputPort<int> input_{"input", context()};

  class Write : public Reaction<InputReadback> {
    using Reaction<InputReadback>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> input_effect{self().input_, context()};
    void handler() final { input_effect.set(11); }
  };

  class Read : public Reaction<InputReadback> {
    using Reaction<InputReadback>::Reaction;
    Trigger<int> input_trigger{self().input_, context()};
    void handler() final {}
  };

  void assemble() final {
    add_reaction<Write>("write");
    add_reaction<Read>("read");
  }
};

class OutputReadback : public Reactor {
public:
  using Reactor::Reactor;

private:
  OutputPort<int> output_{"output", context()};

  class Write : public Reaction<OutputReadback> {
    using Reaction<OutputReadback>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final { output_effect.set(29); }
  };

  class Read : public Reaction<OutputReadback> {
    using Reaction<OutputReadback>::Reaction;
    Trigger<int> output_trigger{self().output_, context()};
    void handler() final {}
  };

  void assemble() final {
    add_reaction<Write>("write");
    add_reaction<Read>("read");
  }
};

// The readback pattern is rejected by the hierarchy rules at declaration
// time: writing the own input and triggering on the own output each violate
// the direction rule on their own.
TEST(ports, RejectsInputPortReadback) {
  TestEnvironment env{};
  InputReadback reactor{"reactor", env.context()};

  try {
    env.execute();
    FAIL() << "expected a ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_NE(std::string{error.what()}.find("Reaction reactor.write may not use reactor.input as an effect"),
              std::string::npos);
  }
}

TEST(ports, RejectsOutputPortReadback) {
  TestEnvironment env{};
  OutputReadback reactor{"reactor", env.context()};

  try {
    env.execute();
    FAIL() << "expected a ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_NE(std::string{error.what()}.find("Reaction reactor.read may not use reactor.output as a trigger"),
              std::string::npos);
  }
}

} // namespace readback

} // namespace xronos::sdk::test
