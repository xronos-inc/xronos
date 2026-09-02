// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cstdint>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

namespace xronos::sdk::test {

TEST(validation, ThrowOnDuplicateReactorNames) {

  class MockReactor : public Reactor {
  public:
    using Reactor::Reactor;
    void assemble() final {}
  };

  TestEnvironment env{};

  auto foo = MockReactor("foo", env.context());
  auto bar = MockReactor("bar", env.context());
  EXPECT_THROW(MockReactor("foo", env.context()), InvalidNameError);
  auto baz = MockReactor("baz", env.context());
  EXPECT_THROW(MockReactor("bar", env.context()), InvalidNameError);
}

TEST(validation, ThrowOnDuplicateElementNames) {
  class BadReactor : public Reactor {
  public:
    using Reactor::Reactor;
    void assemble() final {}

  private:
    InputPort<void> input_{"input", context()};
    OutputPort<void> output_{"input", context()};
  };

  TestEnvironment env{};

  EXPECT_THROW(BadReactor("bad", env.context()), InvalidNameError);

  class TestReactor : public Reactor {
  public:
    using Reactor::context;
    using Reactor::Reactor;

  private:
    class OnStartupReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final {}
    };

    void assemble() final { add_reaction<OnStartupReaction>("startup"); }

    InputPort<void> input_{"input", context()};
    OutputPort<void> output_{"output", context()};
    PeriodicTimer timer_{"timer", context(), std::chrono::seconds{1}};
  };

  TestReactor test{"test", env.context()};
  EXPECT_THROW(PhysicalEvent<void>("timer", test.context()), InvalidNameError);
  EXPECT_THROW(Metric("input", test.context(), "mock", "mock"), InvalidNameError);
  EXPECT_THROW(Startup("timer", test.context()), InvalidNameError);
  EXPECT_THROW(PeriodicTimer("shutdown", test.context(), std::chrono::seconds{1}), InvalidNameError);

  EXPECT_THROW(env.execute(), InvalidNameError);
}

namespace {

// A reactor whose ports a containing reactor may legally reach, used to
// build hierarchy violations one level deeper.
class InnerReactor : public Reactor {
public:
  using Reactor::Reactor;
  auto input() -> InputPort<void>& { return input_; }
  auto output() -> OutputPort<void>& { return output_; }

private:
  void assemble() final {}
  InputPort<void> input_{"input", context()};
  OutputPort<void> output_{"output", context()};
};

} // namespace

TEST(validation, ThrowOnConnectionBeyondDirectChild) {
  class OuterReactor : public Reactor {
  public:
    using Reactor::Reactor;
    auto inner() -> InnerReactor& { return inner_; }

  private:
    void assemble() final {}
    InnerReactor inner_{"inner", context()};
  };

  class SourceReactor : public Reactor {
  public:
    using Reactor::Reactor;
    auto output() -> OutputPort<void>& { return output_; }

  private:
    void assemble() final {}
    OutputPort<void> output_{"output", context()};
  };

  TestEnvironment env{};
  OuterReactor outer{"outer", env.context()};
  SourceReactor source{"source", env.context()};

  // The connection reaches past the direct child into its grandchild, so it
  // is rejected the moment it is declared.
  EXPECT_THROW(env.connect(source.output(), outer.inner().input()), ValidationError);
}

TEST(validation, ThrowOnTriggerBeyondOwnScope) {
  class OuterReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<OuterReactor> {
      using Reaction<OuterReactor>::Reaction;
      // A child's input feeds the child's reactions; the parent may only
      // trigger on the child's outputs.
      Trigger<void> trigger_{self().inner_.input(), context()};
      void handler() final {}
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }
    InnerReactor inner_{"inner", context()};
  };

  TestEnvironment env{};
  OuterReactor outer{"outer", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

TEST(validation, ThrowOnEffectBeyondOwnScope) {
  class OuterReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<OuterReactor> {
      using Reaction<OuterReactor>::Reaction;
      // A child's output belongs to the child's reactions; the parent may
      // only write the child's inputs.
      PortEffect<void> effect_{self().inner_.output(), context()};
      void handler() final {}
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }
    InnerReactor inner_{"inner", context()};
  };

  TestEnvironment env{};
  OuterReactor outer{"outer", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

TEST(validation, ThrowOnDeclareTriggerAfterStart) {
  class TestReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      // Triggers must be declared during assembly; by the time the handler
      // runs the program is sealed.
      void handler() final { Trigger<void>{self().input_, context()}; }
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }

    InputPort<void> input_{"input", context()};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

TEST(validation, ThrowOnDeclarePortEffectAfterStart) {
  class TestReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      // Effects must be declared during assembly; by the time the handler
      // runs the program is sealed.
      void handler() final { PortEffect<void>{self().output_, context()}; }
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }

    OutputPort<void> output_{"output", context()};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

TEST(validation, ThrowOnDeclareProgrammableTimerEffectAfterStart) {
  class TestReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final { ProgrammableTimerEffect<int>{self().timer_, context()}; }
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }

    ProgrammableTimer<int> timer_{"timer", context()};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

TEST(validation, ThrowOnDeclareMetricEffectAfterStart) {
  class TestReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final { MetricEffect{self().metric_, context()}; }
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }

    Metric metric_{"metric", context(), "description", "unit"};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

TEST(validation, ThrowOnSetPortBeforeStart) {
  class TestReactor : public Reactor {
  public:
    TestReactor(std::string_view name, const Context& context)
        : Reactor{name, context} {
      add_reaction<PokeReaction>("poke");
    }
    void poke() { poke_reaction_->poke(); }

  private:
    class PokeReaction : public Reaction<TestReactor> {
    public:
      explicit PokeReaction(const ReactionProperties& properties)
          : Reaction<TestReactor>{properties} {
        self().poke_reaction_ = this;
      }
      void poke() { effect_.set(); }

    private:
      Trigger<void> startup_trigger_{self().startup(), context()};
      PortEffect<void> effect_{self().output_, context()};
      void handler() final {}
    };

    void assemble() final {}
    OutputPort<void> output_{"output", context()};
    PokeReaction* poke_reaction_{nullptr};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  try {
    test.poke();
    FAIL() << "expected ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_STREQ(error.what(), "Port test.output may not be set before execution has started.");
  }
}

TEST(validation, ThrowOnScheduleProgrammableTimerBeforeStart) {
  class TestReactor : public Reactor {
  public:
    TestReactor(std::string_view name, const Context& context)
        : Reactor{name, context} {
      add_reaction<PokeReaction>("poke");
    }
    void poke() { poke_reaction_->poke(); }

  private:
    class PokeReaction : public Reaction<TestReactor> {
    public:
      explicit PokeReaction(const ReactionProperties& properties)
          : Reaction<TestReactor>{properties} {
        self().poke_reaction_ = this;
      }
      void poke() { effect_.schedule(1); }

    private:
      Trigger<void> startup_trigger_{self().startup(), context()};
      ProgrammableTimerEffect<int> effect_{self().timer_, context()};
      void handler() final {}
    };

    void assemble() final {}
    ProgrammableTimer<int> timer_{"timer", context()};
    PokeReaction* poke_reaction_{nullptr};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  try {
    test.poke();
    FAIL() << "expected ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_STREQ(error.what(), "Programmable timer test.timer may not be scheduled before execution has started.");
  }
}

TEST(validation, ThrowOnRecordMetricBeforeStart) {
  class TestReactor : public Reactor {
  public:
    TestReactor(std::string_view name, const Context& context)
        : Reactor{name, context} {
      add_reaction<PokeReaction>("poke");
    }
    void poke_double() { poke_reaction_->poke_double(); }
    void poke_int() { poke_reaction_->poke_int(); }

  private:
    class PokeReaction : public Reaction<TestReactor> {
    public:
      explicit PokeReaction(const ReactionProperties& properties)
          : Reaction<TestReactor>{properties} {
        self().poke_reaction_ = this;
      }
      void poke_double() { effect_.record(1.0); }
      void poke_int() { effect_.record(std::int64_t{1}); }

    private:
      Trigger<void> startup_trigger_{self().startup(), context()};
      MetricEffect effect_{self().metric_, context()};
      void handler() final {}
    };

    void assemble() final {}
    Metric metric_{"metric", context(), "description", "unit"};
    PokeReaction* poke_reaction_{nullptr};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  try {
    test.poke_double();
    FAIL() << "expected ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_STREQ(error.what(), "Metric test.metric may not be recorded before execution has started.");
  }
  try {
    test.poke_int();
    FAIL() << "expected ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_STREQ(error.what(), "Metric test.metric may not be recorded before execution has started.");
  }
}

TEST(validation, ThrowOnTriggerShutdownBeforeStart) {
  class TestReactor : public Reactor {
  public:
    TestReactor(std::string_view name, const Context& context)
        : Reactor{name, context} {
      add_reaction<PokeReaction>("poke");
    }
    void poke() { poke_reaction_->poke(); }

  private:
    class PokeReaction : public Reaction<TestReactor> {
    public:
      explicit PokeReaction(const ReactionProperties& properties)
          : Reaction<TestReactor>{properties} {
        self().poke_reaction_ = this;
      }
      void poke() { effect_.trigger_shutdown(); }

    private:
      Trigger<void> startup_trigger_{self().startup(), context()};
      ShutdownEffect effect_{self().shutdown(), context()};
      void handler() final {}
    };

    void assemble() final {}
    PokeReaction* poke_reaction_{nullptr};
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  try {
    test.poke();
    FAIL() << "expected ValidationError";
  } catch (const ValidationError& error) {
    EXPECT_STREQ(error.what(), "Shutdown test.shutdown may not be triggered before execution has started.");
  }
}

TEST(validation, ThrowOnDeclareShutdownEffectAfterStart) {
  class TestReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class BadReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final { ShutdownEffect{self().shutdown(), context()}; }
    };

    void assemble() final { add_reaction<BadReaction>("bad"); }
  };

  TestEnvironment env{};
  TestReactor test{"test", env.context()};
  EXPECT_THROW(env.execute(), ValidationError);
}

} // namespace xronos::sdk::test
