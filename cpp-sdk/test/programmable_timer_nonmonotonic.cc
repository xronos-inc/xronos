// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// Tests event delivery for a ProgrammableTimer that is scheduled
// non-monotonically. Each frame k arrives at time k * period and schedules
// value k with a delay of delay_periods[k] * period. In consequence, the
// programmable timer triggers at (k + delay_periods[k]) * period. The delay
// of frame 1 drops from 3 to 1, which makes it target an earlier time than
// the pending event from frame 0. The drop from 2 to 1 makes frames 2 and 3
// target the same time, so value 3 overwrites the pending value 2.

#include <array>
#include <cstddef>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

namespace {

constexpr Duration period = 10ms;
constexpr std::array<int, 6> delay_periods{3, 1, 2, 1, 1, 1};

struct Delivery {
  Duration time;
  int value;
};

// Expected deliveries in arrival order.
constexpr std::array<Delivery, 5> expected_deliveries{{
    {2 * period, 1},
    {3 * period, 0},
    {4 * period, 3},
    {5 * period, 4},
    {6 * period, 5},
}};

class FrameSource : public Reactor {
public:
  using Reactor::Reactor;
  [[nodiscard]] auto output() noexcept -> auto& { return output_; }

private:
  PeriodicTimer timer_{"timer", context(), period};
  OutputPort<int> output_{"output", context()};
  int frame_{0};

  class OnTimer : public Reaction<FrameSource> {
    using Reaction<FrameSource>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final { output_effect.set(self().frame_++); }
  };

  void assemble() final { add_reaction<OnTimer>("on_timer"); }
};

class NonMonotonicScheduler : public Reactor {
public:
  using Reactor::Reactor;
  [[nodiscard]] auto input() noexcept -> auto& { return input_; }
  [[nodiscard]] auto output() noexcept -> auto& { return output_; }

private:
  InputPort<int> input_{"input", context()};
  OutputPort<int> output_{"output", context()};
  ProgrammableTimer<int> timer_{"timer", context()};

  class OnFrame : public Reaction<NonMonotonicScheduler> {
    using Reaction<NonMonotonicScheduler>::Reaction;
    Trigger<int> input_trigger{self().input_, context()};
    ProgrammableTimerEffect<int> timer_effect{self().timer_, context()};
    void handler() final {
      auto frame = static_cast<std::size_t>(*input_trigger.get());
      if (frame < delay_periods.size()) {
        timer_effect.schedule(static_cast<int>(frame), delay_periods[frame] * period);
      }
    }
  };

  class OnTimer : public Reaction<NonMonotonicScheduler> {
    using Reaction<NonMonotonicScheduler>::Reaction;
    Trigger<int> timer_trigger{self().timer_, context()};
    PortEffect<int> output_effect{self().output_, context()};
    void handler() final { output_effect.set(*timer_trigger.get()); }
  };

  void assemble() final {
    add_reaction<OnFrame>("on_frame");
    add_reaction<OnTimer>("on_timer");
  }
};

class Receiver : public Reactor {
public:
  using Reactor::Reactor;
  [[nodiscard]] auto input() noexcept -> auto& { return input_; }
  void check_post_conditions() const { EXPECT_EQ(received_, expected_deliveries.size()); }

private:
  InputPort<int> input_{"input", context()};
  std::size_t received_{0};

  class OnValue : public Reaction<Receiver> {
    using Reaction<Receiver>::Reaction;
    Trigger<int> input_trigger{self().input_, context()};
    void handler() final {
      ASSERT_LT(self().received_, expected_deliveries.size());
      const auto& expected = expected_deliveries[self().received_];
      EXPECT_EQ(elapsed_time(), expected.time);
      EXPECT_EQ(*input_trigger.get(), expected.value);
      self().received_++;
    }
  };

  void assemble() final { add_reaction<OnValue>("on_value"); }
};

TEST(programmable_timer_nonmonotonic, DeliversAllEventsAndTerminates) {
  TestEnvironment env{10 * period};
  FrameSource source{"source", env.context()};
  NonMonotonicScheduler scheduler{"scheduler", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(source.output(), scheduler.input());
  env.connect(scheduler.output(), receiver.input());

  env.execute();

  receiver.check_post_conditions();
}

} // namespace

} // namespace xronos::sdk::test
