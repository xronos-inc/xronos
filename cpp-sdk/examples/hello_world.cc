// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>

#include "xronos/sdk.hh"

namespace sdk = xronos::sdk;

class HelloWorld : public sdk::Reactor {
  using sdk::Reactor::Reactor;

  class HelloWorldReaction : public sdk::Reaction<HelloWorld> {
    using sdk::Reaction<HelloWorld>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    void handler() final { std::cout << "Hello, World!\n"; }
  };

  void assemble() final { add_reaction<HelloWorldReaction>("hello"); }
};

auto main() -> int {
  sdk::Environment env{};
  HelloWorld hello_world{"hello_world", env.context()};
  env.execute();
  return 0;
}
