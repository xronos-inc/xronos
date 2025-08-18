// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <string>
#include <vector>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

namespace loops {

class FeedbackVoid : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void chek_post_conditions(int offset) {
    EXPECT_GT(messages_sent_, 0);
    EXPECT_GT(messages_received_, 0);
    EXPECT_EQ(messages_sent_ + offset, messages_received_);
  }

private:
  InputPort<void> input_{"input", context()};
  OutputPort<void> output_{"output", context()};
  PeriodicTimer timer_{"timer", context(), 1s, 0s};

  unsigned messages_sent_{0};
  unsigned messages_received_{0};

  class OnTimer : public Reaction<FeedbackVoid> {
    using Reaction<FeedbackVoid>::Reaction;
    Trigger<void> timer_trigger_{self().timer_, context()};
    PortEffect<void> output_effect_{self().output_, context()};
    void handler() final {
      self().messages_sent_++;
      output_effect_.set();
    }
  };

  class OnInput : public Reaction<FeedbackVoid> {
    using Reaction<FeedbackVoid>::Reaction;
    Trigger<void> input_trigger_{self().input_, context()};
    void handler() final { self().messages_received_++; }
  };

  void assemble() final {
    add_reaction<OnTimer>("on_timer");
    add_reaction<OnInput>("on_input");
  }
};

class FeedthroughVoid : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void chek_post_conditions(int expected) { EXPECT_EQ(messages_received_, expected); }

private:
  InputPort<void> input_{"input", context()};
  OutputPort<void> output_{"output", context()};

  unsigned messages_received_{0};

  class OnInput : public Reaction<FeedthroughVoid> {
    using Reaction<FeedthroughVoid>::Reaction;
    Trigger<void> input_trigger_{self().input_, context()};
    PortEffect<void> output_effect_{self().output_, context()};
    void handler() final {
      self().messages_received_++;
      output_effect_.set();
    }
  };

  void assemble() final { add_reaction<OnInput>("on_input"); }
};

class FeedbackInt : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void chek_post_conditions(int offset) {
    EXPECT_GT(messages_sent_, 0);
    EXPECT_GT(messages_received_, 0);
    EXPECT_EQ(messages_sent_ + offset, messages_received_);
  }

private:
  InputPort<unsigned> input_{"input", context()};
  OutputPort<unsigned> output_{"output", context()};
  PeriodicTimer timer_{"timer", context(), 1s, 0s};

  unsigned messages_sent_{0};
  unsigned messages_received_{0};

  class OnTimer : public Reaction<FeedbackInt> {
    using Reaction<FeedbackInt>::Reaction;
    Trigger<void> timer_trigger_{self().timer_, context()};
    PortEffect<unsigned> output_effect_{self().output_, context()};
    void handler() final {
      self().messages_sent_++;
      output_effect_.set(self().messages_sent_);
    }
  };

  class OnInput : public Reaction<FeedbackInt> {
    using Reaction<FeedbackInt>::Reaction;
    Trigger<unsigned> input_trigger_{self().input_, context()};
    void handler() final {
      self().messages_received_++;
      EXPECT_EQ(self().messages_received_, *input_trigger_.get());
    }
  };

  void assemble() final {
    add_reaction<OnTimer>("on_timer");
    add_reaction<OnInput>("on_input");
  }
};

class FeedthroughInt : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void chek_post_conditions(int expected) { EXPECT_EQ(messages_received_, expected); }

private:
  InputPort<unsigned> input_{"input", context()};
  OutputPort<unsigned> output_{"output", context()};

  unsigned messages_received_{0};

  class OnInput : public Reaction<FeedthroughInt> {
    using Reaction<FeedthroughInt>::Reaction;
    Trigger<unsigned> input_trigger_{self().input_, context()};
    PortEffect<unsigned> output_effect_{self().output_, context()};
    void handler() final {
      self().messages_received_++;
      output_effect_.set(input_trigger_.get());
      EXPECT_EQ(self().messages_received_, *input_trigger_.get());
    }
  };

  void assemble() final { add_reaction<OnInput>("on_input"); }
};

class FeedbackSelfTriggerVoid : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto output() const noexcept -> auto& { return output_; }
  [[nodiscard]] auto input() const noexcept -> auto& { return input_; }

  void chek_post_conditions(int offset) {
    EXPECT_GT(messages_sent_, 0);
    EXPECT_GT(messages_received_, 0);
    EXPECT_EQ(messages_sent_ + offset, messages_received_);
  }

private:
  InputPort<void> input_{"input", context()};
  OutputPort<void> output_{"output", context()};

  unsigned messages_sent_{0};
  unsigned messages_received_{0};

  class OnStartup : public Reaction<FeedbackSelfTriggerVoid> {
    using Reaction<FeedbackSelfTriggerVoid>::Reaction;
    Trigger<void> startup_trigger_{self().startup(), context()};
    PortEffect<void> output_effect_{self().output_, context()};
    void handler() final {
      self().messages_sent_++;
      output_effect_.set();
    }
  };

  class OnInput : public Reaction<FeedbackSelfTriggerVoid> {
    using Reaction<FeedbackSelfTriggerVoid>::Reaction;
    Trigger<void> input_trigger_{self().input_, context()};
    PortEffect<void> output_effect_{self().output_, context()};
    void handler() final {
      self().messages_received_++;
      self().messages_sent_++;
      output_effect_.set();
    }
  };

  void assemble() final {
    add_reaction<OnStartup>("on_startup");
    add_reaction<OnInput>("on_input");
  }
};

TEST(loops, DirectFeedbackLoopVoidNoDelay) {
  TestEnvironment env{5s};
  FeedbackVoid loop{"loop", env.context()};
  env.connect(loop.output(), loop.input());
  env.execute();
  loop.chek_post_conditions(0);
}

TEST(loops, DirectFeedbackLoopIntNoDelay) {
  TestEnvironment env{5s};
  FeedbackInt loop{"loop", env.context()};
  env.connect(loop.output(), loop.input());
  env.execute();
  loop.chek_post_conditions(0);
}

TEST(loops, DirectFeedbackLoopVoidDelayed) {
  TestEnvironment env{5s};
  FeedbackVoid loop{"loop", env.context()};
  env.connect(loop.output(), loop.input(), 500ms);
  env.execute();
  loop.chek_post_conditions(-1);
}

TEST(loops, DirectFeedbackLoopIntDelayed) {
  TestEnvironment env{5s};
  FeedbackInt loop{"loop", env.context()};
  env.connect(loop.output(), loop.input(), 500ms);
  env.execute();
  loop.chek_post_conditions(-1);
}

TEST(loops, DirectFeedbackLoopVoidDelayed2) {
  TestEnvironment env{5s};
  FeedbackVoid loop{"loop", env.context()};
  env.connect(loop.output(), loop.input(), 2s);
  env.execute();
  loop.chek_post_conditions(-2);
}

TEST(loops, DirectFeedbackLoopIntDelayed2) {
  TestEnvironment env{5s};
  FeedbackInt loop{"loop", env.context()};
  env.connect(loop.output(), loop.input(), 2s);
  env.execute();
  loop.chek_post_conditions(-2);
}

TEST(loops, MultiStageFeedbackLoopVoidNoDelay) {
  TestEnvironment env{5s};
  FeedbackVoid loop{"loop", env.context()};
  std::vector<std::unique_ptr<FeedthroughVoid>> stages;
  constexpr unsigned num_stages{10};
  for (unsigned i{0}; i < num_stages; i++) {
    stages.emplace_back(std::make_unique<FeedthroughVoid>("stage" + std::to_string(i), env.context()));
  }
  env.connect(loop.output(), stages.front()->input());
  for (unsigned i{0}; i < num_stages - 1; i++) {
    env.connect(stages[i]->output(), stages[i + 1]->input());
  }
  env.connect(stages.back()->output(), loop.input());
  env.execute();
  loop.chek_post_conditions(0);
  for (auto& stage : stages) {
    stage->chek_post_conditions(6);
  }
}

TEST(loops, MultiStageFeedbackLoopIntNoDelay) {
  TestEnvironment env{5s};
  FeedbackInt loop{"loop", env.context()};
  std::vector<std::unique_ptr<FeedthroughInt>> stages;
  constexpr unsigned num_stages{10};
  for (unsigned i{0}; i < num_stages; i++) {
    stages.emplace_back(std::make_unique<FeedthroughInt>("stage" + std::to_string(i), env.context()));
  }
  env.connect(loop.output(), stages.front()->input());
  for (unsigned i{0}; i < num_stages - 1; i++) {
    env.connect(stages[i]->output(), stages[i + 1]->input());
  }
  env.connect(stages.back()->output(), loop.input());
  env.execute();
  loop.chek_post_conditions(0);
  for (auto& stage : stages) {
    stage->chek_post_conditions(6);
  }
}

TEST(loops, MultiStageFeedbackLoopVoidDelayed) {
  TestEnvironment env{100s};
  FeedbackVoid loop{"loop", env.context()};
  std::vector<std::unique_ptr<FeedthroughVoid>> stages;
  constexpr unsigned num_stages{10};
  for (unsigned i{0}; i < num_stages; i++) {
    stages.emplace_back(std::make_unique<FeedthroughVoid>("stage" + std::to_string(i), env.context()));
  }
  env.connect(loop.output(), stages.front()->input(), 1s);
  for (unsigned i{0}; i < num_stages - 1; i++) {
    env.connect(stages[i]->output(), stages[i + 1]->input(), 1s);
  }
  env.connect(stages.back()->output(), loop.input(), 1s);
  env.execute();
  loop.chek_post_conditions(-11);
  for (unsigned i{0}; i < num_stages; i++) {
    stages[i]->chek_post_conditions(100 - i);
  }
}

TEST(loops, MultiStageFeedbackLoopIntDelayed) {
  TestEnvironment env{100s};
  FeedbackInt loop{"loop", env.context()};
  std::vector<std::unique_ptr<FeedthroughInt>> stages;
  constexpr unsigned num_stages{10};
  for (unsigned i{0}; i < num_stages; i++) {
    stages.emplace_back(std::make_unique<FeedthroughInt>("stage" + std::to_string(i), env.context()));
  }
  env.connect(loop.output(), stages.front()->input(), 1s);
  for (unsigned i{0}; i < num_stages - 1; i++) {
    env.connect(stages[i]->output(), stages[i + 1]->input(), 1s);
  }
  env.connect(stages.back()->output(), loop.input(), 1s);
  env.execute();
  loop.chek_post_conditions(-11);
  for (unsigned i{0}; i < num_stages; i++) {
    stages[i]->chek_post_conditions(100 - i);
  }
}

TEST(loops, DirectFeedbackSelfTriggered) {
  TestEnvironment env{5s};
  FeedbackSelfTriggerVoid loop{"loop", env.context()};
  env.connect(loop.output(), loop.input(), 1s);
  env.execute();
  loop.chek_post_conditions(-1);
}

} // namespace loops

} // namespace xronos::sdk::test
