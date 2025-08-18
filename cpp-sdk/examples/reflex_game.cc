// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <random>
#include <string_view>
#include <thread>

#include "xronos/sdk.hh"

namespace sdk = xronos::sdk;

class RandomDelay : public sdk::Reactor {

private:
  sdk::InputPort<void> input_{"input", context()};
  sdk::OutputPort<void> output_{"output", context()};
  sdk::ProgrammableTimer<void> delay_{"delay", context()};

  std::mt19937 rand{
      std::random_device{}()}; // NOTE: originally used get_physical_time().time_since_epoch().count() as seed
  std::uniform_int_distribution<std::uint64_t> dist;

public:
  // Constructor taking duration parameters
  RandomDelay(std::string_view name, sdk::Context context, std::chrono::milliseconds min_delay,
              std::chrono::milliseconds max_delay)
      : sdk::Reactor(name, context)
      , dist(1, (max_delay - min_delay) / std::chrono::milliseconds(1)) {}

  auto input() -> sdk::InputPort<void>& { return input_; }
  auto output() -> sdk::OutputPort<void>& { return output_; }

  class Output : public sdk::Reaction<RandomDelay> {
    using sdk::Reaction<RandomDelay>::Reaction;
    Trigger<void> delay_trigger{self().delay_, context()};
    PortEffect<void> output_effect{self().output_, context()};
    void handler() final { output_effect.set(); }
  };

  class Schedule : public sdk::Reaction<RandomDelay> {
    using sdk::Reaction<RandomDelay>::Reaction;
    Trigger<void> input_trigger{self().input_, context()};
    ProgrammableTimerEffect<void> delay_effect{self().delay_, context()};
    void handler() final { delay_effect.schedule(self().dist(self().rand) * std::chrono::milliseconds(1)); }
  };

  void assemble() final {
    add_reaction<Output>("output");
    add_reaction<Schedule>("schedule");
  }
};

class KeyboardInput : public sdk::Reactor {
private:
  sdk::PhysicalEvent<int> keyboard_input_{"keyboard_input", context()};
  sdk::OutputPort<void> enter_{"enter", context()};
  sdk::OutputPort<void> quit_{"quit", context()};

  std::thread thread_;
  std::atomic<bool> terminate_{false};

public:
  KeyboardInput(std::string_view name, sdk::Context context)
      : sdk::Reactor(name, context) {}

  auto quit() -> sdk::OutputPort<void>& { return quit_; }
  auto enter() -> sdk::OutputPort<void>& { return enter_; }

  // Startup reaction (spawns input thread)
  class Startup : public sdk::Reaction<KeyboardInput> {
    using sdk::Reaction<KeyboardInput>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    void handler() final {
      self().thread_ = std::thread([&]() {
        int key{0};
        while (!self().terminate_.load()) {
          key = getchar();
          self().keyboard_input_.trigger(key);
        }
      });
    }
  };

  // Input handling reaction
  class Input : public sdk::Reaction<KeyboardInput> {
    using sdk::Reaction<KeyboardInput>::Reaction;
    Trigger<int> input_trigger{self().keyboard_input_, context()};
    PortEffect<void> enter_effect{self().enter_, context()};
    PortEffect<void> quit_effect{self().quit_, context()};

    void handler() final {
      int key = *input_trigger.get();
      if (key == '\n') {
        enter_effect.set();
      } else if (key == EOF) {
        quit_effect.set();
      }
    }
  };

  // Shutdown reaction (cleans up thread)
  class Shutdown : public sdk::Reaction<KeyboardInput> {
    using sdk::Reaction<KeyboardInput>::Reaction;
    Trigger<void> shutdown_trigger{self().shutdown(), context()};
    void handler() final {
      self().terminate_.store(true);
      if (self().thread_.joinable()) {
        self().thread_.join();
      }
    }
  };

  void assemble() final {
    add_reaction<Startup>("startup");
    add_reaction<Input>("input");
    add_reaction<Shutdown>("shutdown");
  }
};

class GameLogic : public sdk::Reactor {
private:
  // Port declarations
  sdk::OutputPort<void> request_prompt_{"request_prompt", context()};
  sdk::InputPort<void> prompt_{"prompt", context()};
  sdk::InputPort<void> enter_{"enter", context()};
  sdk::InputPort<void> quit_{"quit", context()};

  // State variables
  sdk::TimePoint prompt_time_{sdk::TimePoint::min()};
  unsigned count_{0};
  std::chrono::milliseconds total_time_{0};

public:
  GameLogic(std::string_view name, sdk::Context context)
      : sdk::Reactor(name, context) {}

  auto request_prompt() -> sdk::OutputPort<void>& { return request_prompt_; }
  auto prompt() -> sdk::InputPort<void>& { return prompt_; }
  auto enter() -> sdk::InputPort<void>& { return enter_; }
  auto quit() -> sdk::InputPort<void>& { return quit_; }

  class Startup : public sdk::Reaction<GameLogic> {
    using sdk::Reaction<GameLogic>::Reaction;
    PortEffect<void> request_effect{self().request_prompt(), context()};
    Trigger<void> startup_trigger{self().startup(), context()};

    void handler() final {
      std::cout << "***********************************************\n"
                << "Watch for the prompt, then hit Return or Enter.\n"
                << "Type Control-D (EOF) to quit.\n\n";
      request_effect.set();
    }
  };

  class Prompt : public sdk::Reaction<GameLogic> {
    using sdk::Reaction<GameLogic>::Reaction;
    Trigger<void> prompt_trigger{self().prompt(), context()};
    void handler() final {
      self().prompt_time_ = self().get_time() + self().get_lag();
      std::cout << "\nHit Return or Enter!\n";
    }
  };

  class Response : public sdk::Reaction<GameLogic> {
    using sdk::Reaction<GameLogic>::Reaction;
    PortEffect<void> request_effect{self().request_prompt(), context()};
    Trigger<void> enter_trigger{self().enter(), context()};
    void handler() final {
      if (self().prompt_time_ == sdk::TimePoint::min()) {
        std::cout << "YOU CHEATED!\n";
        self().request_shutdown();
      } else {
        auto elapsed = self().get_time() - self().prompt_time_;
        auto time_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        std::cout << "Response time: " << time_in_ms.count() << "ms\n";
        self().count_++;
        self().total_time_ += time_in_ms;
        self().prompt_time_ = sdk::TimePoint::min();
        request_effect.set();
      }
    }
  };

  class Quit : public sdk::Reaction<GameLogic> {
    using sdk::Reaction<GameLogic>::Reaction;
    Trigger<void> quit_trigger{self().quit_, context()};
    void handler() final { self().request_shutdown(); }
  };

  void assemble() final {
    add_reaction<Startup>("startup");
    add_reaction<Prompt>("prompt");
    add_reaction<Response>("response");
    add_reaction<Quit>("quit");
  }
};

auto constexpr MIN_DELAY = std::chrono::seconds(2);
auto constexpr MAX_DELAY = std::chrono::seconds(5);

auto main() -> int {
  sdk::Environment env{}; // Create a runtime environment

  // Instantiate reactors
  RandomDelay delay{"delay", env.context(), MIN_DELAY, MAX_DELAY};
  KeyboardInput keyboard{"keyboard", env.context()};
  GameLogic logic{"logic", env.context()};

  // Connect them
  env.connect(logic.request_prompt(), delay.input());
  env.connect(delay.output(), logic.prompt());
  env.connect(keyboard.enter(), logic.enter());
  env.connect(keyboard.quit(), logic.quit());

  env.execute(); // Run the reactor system
  return 0;
}
