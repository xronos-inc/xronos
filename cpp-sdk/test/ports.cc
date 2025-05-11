// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

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
      self().reaction_executed_at_ = self().get_time_since_startup();
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
      self().reaction_executed_at_ = self().get_time_since_startup();
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

  void check_post_conditions(int expected_count) {
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

} // namespace xronos::sdk::test
