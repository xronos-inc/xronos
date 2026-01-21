// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "xronos/sdk.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/periodic_timer.hh"
#include "xronos/sdk/port.hh"
#include "xronos/sdk/programmable_timer.hh"
#include "xronos/sdk/reactor.hh"
#include "xronos/sdk/time.hh"
#include "xronos/util/logging.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

TEST(startup_and_shutdown, StartupTriggersOnce) {
  class StartupTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> bool { return reaction_executed_; }

  private:
    bool reaction_executed_{false};

    class OnStartupReaction : public Reaction<StartupTest> {
      using Reaction<StartupTest>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final {
        EXPECT_FALSE(self().reaction_executed_);
        EXPECT_TRUE(startup_trigger_.is_present());
        self().reaction_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnStartupReaction>("on_startup"); }
  };

  TestEnvironment env{};
  StartupTest startup_test{"startup_test", env.context()};
  env.execute();

  EXPECT_TRUE(startup_test.reaction_executed());
}

TEST(startup_and_shutdown, ShutdownTriggersOnce) {
  class ShutdownTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> bool { return reaction_executed_; }

  private:
    bool reaction_executed_{false};

    class OnShutdownReaction : public Reaction<ShutdownTest> {
      using Reaction<ShutdownTest>::Reaction;
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final {
        EXPECT_FALSE(self().reaction_executed_);
        EXPECT_TRUE(shutdown_trigger_.is_present());
        self().reaction_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnShutdownReaction>("on_shutdown"); }
  };

  TestEnvironment env{};
  ShutdownTest shutdown_test{"shutdown_test", env.context()};
  env.execute();

  EXPECT_TRUE(shutdown_test.reaction_executed());
}

TEST(startup_and_shutdown, TestStartupAndShutdown) {
  class StartupAndShutdownTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> int { return reaction_executed_; }

  private:
    int reaction_executed_{0};
    TimePoint startup_time_{};

    class TestReaction : public Reaction<StartupAndShutdownTest> {
      using Reaction<StartupAndShutdownTest>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final {
        EXPECT_TRUE(startup_trigger_.is_present() || shutdown_trigger_.is_present());
        EXPECT_FALSE(startup_trigger_.is_present() && shutdown_trigger_.is_present());
        if (startup_trigger_.is_present()) {
          self().startup_time_ = self().get_time();
          EXPECT_EQ(self().reaction_executed_, 0);
        }
        if (shutdown_trigger_.is_present()) {
          EXPECT_EQ(self().reaction_executed_, 1);
          EXPECT_EQ(self().startup_time_, self().get_time()); // no time passes between startup and shutdown
        }
        self().reaction_executed_++;
      }
    };

    void assemble() final { add_reaction<TestReaction>("on_startup_or_shutdown"); }
  };

  TestEnvironment env{};
  StartupAndShutdownTest shutdown_test{"startup_and_shutdown_test", env.context()};
  env.execute();

  EXPECT_EQ(shutdown_test.reaction_executed(), 2);
}

TEST(startup_and_shutdown, TestShutdownWithEffect) {
  class StartupAndShutdownTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> int { return reaction_executed_; }

  private:
    int reaction_executed_{0};
    TimePoint startup_time_{};

    OutputPort<void> output_{"output", context()};

    class TestReaction : public Reaction<StartupAndShutdownTest> {
      using Reaction<StartupAndShutdownTest>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      PortEffect<void> output_effect_{self().output_, context()};
      void handler() final { self().reaction_executed_++; }
    };

    void assemble() final { add_reaction<TestReaction>("on_startup_or_shutdown"); }
  };

  TestEnvironment env{};
  StartupAndShutdownTest shutdown_test{"startup_and_shutdown_test", env.context()};

  std::stringstream capture{};
  util::log::detail::Logger::set_ostream(capture);

  EXPECT_THROW(env.execute(), ValidationError);

  EXPECT_EQ(shutdown_test.reaction_executed(), 0);

  std::string captured_output = capture.str();
  EXPECT_FALSE(captured_output.empty());
  EXPECT_NE(captured_output.find("Reactions triggered by shutdown may not have any effects"), std::string::npos);
}

TEST(startup_and_shutdown, TestStarvation) {
  class StarvationTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto shutdown_reaction_executed() const noexcept -> bool { return shutdown_reaction_executed_; }

  private:
    ProgrammableTimer<void> programmable_timer_{"timer", context()};
    bool shutdown_reaction_executed_{false};

    class OnStartupReaction : public Reaction<StarvationTest> {
      using Reaction<StarvationTest>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      ProgrammableTimerEffect<void> timer_effect_{self().programmable_timer_, context()};
      void handler() final { timer_effect_.schedule(1s); }
    };

    class OnTimerReaction : public Reaction<StarvationTest> {
      using Reaction<StarvationTest>::Reaction;
      Trigger<void> _{self().programmable_timer_, context()};
      void handler() final {}
    };

    class OnShutdownReaction : public Reaction<StarvationTest> {
      using Reaction<StarvationTest>::Reaction;
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final {
        EXPECT_TRUE(shutdown_trigger_.is_present());
        EXPECT_EQ(self().get_time_since_startup(), 1s);
        EXPECT_FALSE(self().shutdown_reaction_executed_);
        self().shutdown_reaction_executed_ = true;
      }
    };

    void assemble() final {
      add_reaction<OnStartupReaction>("on_startup");
      add_reaction<OnTimerReaction>("on_timer");
      add_reaction<OnShutdownReaction>("on_shutdown");
    }
  };

  TestEnvironment env{};
  StarvationTest test{"test", env.context()};
  env.execute();

  EXPECT_TRUE(test.shutdown_reaction_executed());
}

struct TriggerShutdownTestReactorParameters {
  std::string name;
  Duration period;
  unsigned max_count;
  bool has_shutdown_effect;
};

struct TestTriggerShutdownParameters {
  std::vector<TriggerShutdownTestReactorParameters> reactor_parameters;
  Duration expected_shutdown_time;
};

class TestTriggerShutdown : public ::testing::TestWithParam<TestTriggerShutdownParameters> {};

TEST_P(TestTriggerShutdown, run) {
  class TriggerShutdownTestReactor : public Reactor {
  public:
    TriggerShutdownTestReactor(std::string_view name, Context parent_context, Duration period, unsigned max_count,
                               bool has_shutdown_effect)
        : Reactor{name, parent_context}
        , timer_{"timer", context(), period, period}
        , max_count_{max_count}
        , has_shutdown_effect_{has_shutdown_effect} {}

    [[nodiscard]] auto shutdown_reaction_executed() const noexcept -> bool { return shutdown_reaction_executed_; }
    [[nodiscard]] auto shutdown_time() const noexcept -> Duration { return shutdown_time_; }

  private:
    PeriodicTimer timer_;

    bool shutdown_reaction_executed_{false};
    Duration shutdown_time_{Duration::zero()};
    unsigned count_{0};
    unsigned max_count_;
    bool has_shutdown_effect_{false};

    class OnTimerReactionWithShutdownEffect : public Reaction<TriggerShutdownTestReactor> {
      using Reaction<TriggerShutdownTestReactor>::Reaction;
      Trigger<void> _{self().timer_, context()};
      ShutdownEffect shutdown_effect{self().shutdown(), context()};

      void handler() final {
        EXPECT_FALSE(self().shutdown_reaction_executed_);
        EXPECT_LT(self().count_, self().max_count_);
        self().count_++;
        if (self().count_ == self().max_count_) {
          shutdown_effect.trigger_shutdown();
        }
      }
    };

    class OnTimerReactionWithoutShutdownEffect : public Reaction<TriggerShutdownTestReactor> {
      using Reaction<TriggerShutdownTestReactor>::Reaction;
      Trigger<void> _{self().timer_, context()};

      void handler() final {
        EXPECT_FALSE(self().shutdown_reaction_executed_);
        EXPECT_LT(self().count_, self().max_count_);
        self().count_++;
      }
    };

    class OnShutdownReaction : public Reaction<TriggerShutdownTestReactor> {
      using Reaction<TriggerShutdownTestReactor>::Reaction;
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final {
        EXPECT_TRUE(shutdown_trigger_.is_present());
        EXPECT_FALSE(self().shutdown_reaction_executed_);
        self().shutdown_reaction_executed_ = true;
        self().shutdown_time_ = self().get_time_since_startup();
      }
    };

    void assemble() final {
      if (has_shutdown_effect_) {
        add_reaction<OnTimerReactionWithShutdownEffect>("on_timer");
      } else {
        add_reaction<OnTimerReactionWithoutShutdownEffect>("on_timer");
      }
      add_reaction<OnShutdownReaction>("on_shutdown");
    }
  };

  TestEnvironment env{};

  std::vector<std::unique_ptr<TriggerShutdownTestReactor>> reactors;
  for (const auto& params : GetParam().reactor_parameters) {
    reactors.emplace_back(std::make_unique<TriggerShutdownTestReactor>(params.name, env.context(), params.period,
                                                                       params.max_count, params.has_shutdown_effect));
  }

  env.execute();

  for (auto& reactor : reactors) {
    std::cout << "checking reactor " << reactor->fqn() << '\n';
    EXPECT_TRUE(reactor->shutdown_reaction_executed());
    EXPECT_EQ(reactor->shutdown_time(), GetParam().expected_shutdown_time);
  }
}

INSTANTIATE_TEST_SUITE_P(
    startup_and_shutdown, TestTriggerShutdown,
    ::testing::Values(
        TestTriggerShutdownParameters{
            .reactor_parameters = {TriggerShutdownTestReactorParameters{"test", 100ms, 10, true}},
            .expected_shutdown_time = 1s},
        TestTriggerShutdownParameters{
            .reactor_parameters = {TriggerShutdownTestReactorParameters{"test1", 10ms, 100, true},
                                   TriggerShutdownTestReactorParameters{"test2", 100ms, 10, true}},
            .expected_shutdown_time = 1s},
        TestTriggerShutdownParameters{
            .reactor_parameters = {TriggerShutdownTestReactorParameters{"test1", 100ms, 100, true},
                                   TriggerShutdownTestReactorParameters{"test2", 100ms, 10, true}},
            .expected_shutdown_time = 1s},
        TestTriggerShutdownParameters{.reactor_parameters =
                                          {
                                              TriggerShutdownTestReactorParameters{"test1", 1ms, 100, true},
                                              TriggerShutdownTestReactorParameters{"test2", 2ms, 100, true},
                                              TriggerShutdownTestReactorParameters{"test3", 3ms, 50, true},
                                              TriggerShutdownTestReactorParameters{"test4", 4ms, 50, true},
                                              TriggerShutdownTestReactorParameters{"test5", 5ms, 20, true},
                                          },
                                      .expected_shutdown_time = 100ms},
        TestTriggerShutdownParameters{.reactor_parameters =
                                          {
                                              TriggerShutdownTestReactorParameters{"test1", 1ms, 100, true},
                                              TriggerShutdownTestReactorParameters{"test2", 2ms, 100, true},
                                              TriggerShutdownTestReactorParameters{"test3", 3ms, 20, true},
                                              TriggerShutdownTestReactorParameters{"test4", 4ms, 50, true},
                                              TriggerShutdownTestReactorParameters{"test5", 5ms, 20, true},
                                          },
                                      .expected_shutdown_time = 60ms},
        TestTriggerShutdownParameters{.reactor_parameters =
                                          {
                                              TriggerShutdownTestReactorParameters{"test1", 1ms, 1000, false},
                                              TriggerShutdownTestReactorParameters{"test2", 2ms, 100, true},
                                              TriggerShutdownTestReactorParameters{"test3", 3ms, 50, true},
                                              TriggerShutdownTestReactorParameters{"test4", 4ms, 1000, false},
                                              TriggerShutdownTestReactorParameters{"test5", 5ms, 1000, false},
                                          },
                                      .expected_shutdown_time = 150ms}

        ));

} // namespace xronos::sdk::test
