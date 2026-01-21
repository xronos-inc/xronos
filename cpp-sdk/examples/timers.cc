// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono> // IWYU pragma: keep
#include <iostream>

#include "xronos/sdk.hh"

namespace sdk = xronos::sdk;

using namespace std::literals::chrono_literals;

class Timers : public sdk::Reactor {
  using sdk::Reactor::Reactor;

  sdk::PeriodicTimer timer1_{"timer1", context(), 1s};
  sdk::PeriodicTimer timer2_{"timer2", context(), 1s, 500ms};

  class OnTimerReaction : public sdk::Reaction<Timers> {
    using sdk::Reaction<Timers>::Reaction;

    Trigger<void> timer1_trigger{self().timer1_, context()};
    Trigger<void> timer2_trigger{self().timer2_, context()};

    void handler() final {
      if (timer1_trigger.is_present()) {
        std::cout << "timer1 triggered\n";
      }
      if (timer2_trigger.is_present()) {
        std::cout << "timer2 triggered\n";
      }
    }
  };

  void assemble() final { add_reaction<OnTimerReaction>("on_timer"); }
};

auto main() -> int {
  sdk::Environment env{};
  Timers timers{"timers", env.context()};
  env.execute();
  return 0;
}
