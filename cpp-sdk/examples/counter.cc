// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono> // IWYU pragma: keep
#include <cstdint>
#include <iostream>

#include "xronos/sdk.hh"

namespace sdk = xronos::sdk;
using namespace std::literals::chrono_literals;

class Counter : public sdk::Reactor {
public:
  using sdk::Reactor::Reactor;
  auto output() -> sdk::OutputPort<std::uint64_t>& { return output_; }

private:
  sdk::ProgrammableTimer<std::uint64_t> count_{"count", context()};
  sdk::OutputPort<std::uint64_t> output_{"output", context()};

  class CountReaction : public sdk::Reaction<Counter> {
    using sdk::Reaction<Counter>::Reaction;

    Trigger<void> startup_trigger{self().startup(), context()};
    Trigger<std::uint64_t> count_trigger{self().count_, context()};
    PortEffect<std::uint64_t> output_effect{self().output_, context()};
    ProgrammableTimerEffect<std::uint64_t> count_effect{self().count_, context()};

    void handler() final {
      std::uint64_t count = startup_trigger.is_present() ? 1UL : *count_trigger.get() + 1;
      output_effect.set(count);
      count_effect.schedule(count, 1s);
    }
  };

  void assemble() final { add_reaction<CountReaction>("count"); }
};

template <class T> class Printer : public sdk::Reactor {
public:
  using sdk::Reactor::Reactor;
  auto input() -> sdk::InputPort<T>& { return input_; }

private:
  sdk::InputPort<std::uint64_t> input_{"input", context()};

  class PrintReaction : public sdk::Reaction<Printer<T>> {
    using sdk::Reaction<Printer<T>>::Reaction;
    sdk::BaseReaction::Trigger<T> input_trigger{this->self().input_, this->context()};
    void handler() final { std::cout << "Received: " << *input_trigger.get() << '\n'; }
  };

  void assemble() final { add_reaction<PrintReaction>("print"); }
};

auto main() -> int {
  sdk::Environment env{};
  Counter counter{"counter", env.context()};
  Printer<std::uint64_t> printer{"printer", env.context()};
  env.connect(counter.output(), printer.input());
  env.execute();
  return 0;
}
