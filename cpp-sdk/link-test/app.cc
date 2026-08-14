// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// One of the translation units of the SDK link test (see CMakeLists.txt).
// Together with main.cc it forms the executable: two translation units that
// both include every public header, so a header definition missing `inline`
// is emitted as a duplicate strong symbol and fails the executable's link.

#include "include_all.hh"

#include <string>

namespace xronos::sdk::link_test {

namespace {

class Echo final : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto done() const noexcept -> bool { return done_; }

private:
  ProgrammableTimer<std::string> timer_{"timer", context()};

  bool done_{false};

  class Start : public Reaction<Echo> {
    using Reaction<Echo>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    ProgrammableTimerEffect<std::string> timer_effect{self().timer_, context()};
    void handler() final { timer_effect.schedule(std::string{"ping"}); }
  };

  class OnTimer : public Reaction<Echo> {
    using Reaction<Echo>::Reaction;
    Trigger<std::string> timer_trigger{self().timer_, context()};
    void handler() final {
      const auto value = timer_trigger.get();
      self().done_ = value != nullptr && *value == "ping";
    }
  };

  void assemble() final {
    add_reaction<Start>("start");
    add_reaction<OnTimer>("on_timer");
  }
};

} // namespace

auto run_app_program() -> bool {
  TestEnvironment env{};
  Echo echo{"echo", env.context()};
  env.execute();
  return echo.done();
}

} // namespace xronos::sdk::link_test
