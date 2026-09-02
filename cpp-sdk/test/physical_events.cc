// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include "xronos/sdk.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/physical_event.hh"
#include "xronos/sdk/reaction.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/value.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

// A reactor whose physical event triggers a reaction; the handler counts
// its executions and requests shutdown at the given count.
class StopOnEvent : public Reactor {
public:
  StopOnEvent(std::string_view name, Context parent_context, unsigned shutdown_at_count = 1)
      : Reactor{name, parent_context}
      , shutdown_at_count_{shutdown_at_count} {}

  [[nodiscard]] auto event() noexcept -> PhysicalEvent<int>& { return event_; }
  [[nodiscard]] auto executions() const noexcept -> unsigned { return executions_; }

private:
  PhysicalEvent<int> event_{"event", context()};
  unsigned executions_{0};
  unsigned shutdown_at_count_;

  class OnEvent : public Reaction<StopOnEvent> {
    using Reaction<StopOnEvent>::Reaction;
    Trigger<int> event_trigger_{self().event_, context()};
    ShutdownEffect shutdown_effect_{self().shutdown(), context()};
    void handler() final {
      self().executions_++;
      if (self().executions_ == self().shutdown_at_count_) {
        shutdown_effect_.trigger_shutdown();
      }
    }
  };

  void assemble() final { add_reaction<OnEvent>("on_event"); }
};

// The trigger reports NotStarted in the short span between the start of
// execute() and the runtime opening its live window; retrying absorbs that
// span. Gives up after five seconds and returns the last status.
auto trigger_until_accepted(PhysicalEvent<int>& event) -> TriggerStatus {
  auto status = event.trigger(0);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (status != TriggerStatus::Accepted && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
    status = event.trigger(0);
  }
  return status;
}

TEST(physical_events, NotStartedBeforeExecute) {
  TestEnvironment env{};
  StopOnEvent reactor{"reactor", env.context()};
  EXPECT_EQ(reactor.event().trigger(1), TriggerStatus::NotStarted);
}

TEST(physical_events, ThrowsOnEmptyPayload) {
  TestEnvironment env{};
  StopOnEvent reactor{"reactor", env.context()};

  // An empty payload is a caller error.
  EXPECT_THROW(std::ignore = reactor.event().trigger(Value<int>{}), std::invalid_argument);
  EXPECT_THROW(std::ignore = reactor.event().trigger(ValueView<int>{}), std::invalid_argument);
}

TEST(physical_events, AcceptedDuringRun) {
  // The timeout only bounds the test if the trigger is never accepted; the
  // accepted trigger ends the run through the shutdown reaction.
  TestEnvironment env{10s};
  StopOnEvent reactor{"reactor", env.context()};

  std::thread run_thread{[&env]() { env.execute(); }};
  auto status = trigger_until_accepted(reactor.event());
  run_thread.join();

  EXPECT_EQ(status, TriggerStatus::Accepted);
  EXPECT_EQ(reactor.executions(), 1U);
}

TEST(physical_events, TriggersDrainAcrossEndOfExecution) {
  // The interesting interleavings -- a trigger in flight while execute()
  // returns, and attempts arriving just after -- are only a few
  // instructions wide, so the race is retried many times.
  constexpr unsigned repetitions = 100;

  for (unsigned repetition = 0; repetition < repetitions; repetition++) {
    SCOPED_TRACE(repetition);
    bool only_sane_statuses = true;
    bool observed_accepted = false;
    {
      TestEnvironment env{10s};
      StopOnEvent reactor{"reactor", env.context()};

      // The worker hammers the trigger from before the program starts until
      // after it ends; the first accepted fire ends the run through the
      // shutdown reaction. The worker only records; the checks run after
      // the join.
      std::thread worker{[&reactor, &only_sane_statuses, &observed_accepted]() {
        while (true) {
          auto status = reactor.event().trigger(0);
          if (status == TriggerStatus::Stopped) {
            break;
          }
          if (status == TriggerStatus::Accepted) {
            observed_accepted = true;
          } else if (status != TriggerStatus::NotStarted) {
            only_sane_statuses = false;
          }
        }
      }};

      env.execute();
      // execute() has returned, so every further attempt reports Stopped:
      // the worker is guaranteed to exit, and joins before the environment
      // leaves this scope.
      worker.join();
    }
    EXPECT_TRUE(only_sane_statuses);
    // The run ends through the shutdown reaction, so a fire was accepted;
    // without this the repetition would not have raced a live delivery
    // against the end of execution at all.
    EXPECT_TRUE(observed_accepted);
  }
}

TEST(physical_events, BackToBackTriggersAllExecute) {
  // A coarse clock can hand consecutive triggers the same clock reading, so
  // back-to-back triggers stress the runtime's duty to stamp each event with
  // its own tag; a tag collision would silently drop an earlier event. The
  // handler requests shutdown on the last expected execution, so the run
  // ends only after every event was processed. The checks run after the
  // join; a failed trigger surfaces as a short status list or a low count.
  constexpr unsigned num_triggers = 100;
  TestEnvironment env{10s};
  StopOnEvent reactor{"reactor", env.context(), num_triggers};

  std::thread run_thread{[&env]() { env.execute(); }};
  auto first_status = trigger_until_accepted(reactor.event());
  std::vector<TriggerStatus> statuses;
  for (unsigned i = 1; i < num_triggers; i++) {
    statuses.push_back(reactor.event().trigger(static_cast<int>(i)));
  }
  run_thread.join();

  EXPECT_EQ(first_status, TriggerStatus::Accepted);
  for (auto status : statuses) {
    EXPECT_EQ(status, TriggerStatus::Accepted);
  }
  EXPECT_EQ(reactor.executions(), num_triggers);
}

TEST(physical_events, StoppedAfterExecute) {
  TestEnvironment env{10s};
  StopOnEvent reactor{"reactor", env.context()};

  std::thread run_thread{[&env]() { env.execute(); }};
  auto accepted = trigger_until_accepted(reactor.event());
  run_thread.join();
  ASSERT_EQ(accepted, TriggerStatus::Accepted);

  EXPECT_EQ(reactor.event().trigger(2), TriggerStatus::Stopped);
  EXPECT_EQ(reactor.executions(), 1U);
}

} // namespace xronos::sdk::test
