// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// NOTE: These tests assert on execute() being called with *specific* runtime
// providers. Test harnesses that rewrite execute() calls in this suite to
// substitute their own runtime provider must exclude this file.

#include <memory>
#include <string_view>

#include "xronos/runtime/interfaces.hh"
#include "xronos/sdk.hh"
#include "gtest/gtest.h"

namespace xronos::sdk::test {

namespace {

// A provider whose version can never match the SDK's: execute() must reject
// it before ever asking it for a runtime.
class MismatchedProvider final : public RuntimeProvider {
  [[nodiscard]] auto get_runtime() const noexcept -> std::unique_ptr<runtime::Runtime> final { return nullptr; }
  [[nodiscard]] auto version() const noexcept -> std::string_view final { return "0.0.0+mismatch"; }
};

class CountingReactor final : public Reactor {
public:
  using Reactor::Reactor;

  [[nodiscard]] auto startups_handled() const noexcept -> unsigned { return startups_handled_; }

private:
  unsigned startups_handled_{0};

  class OnStartup : public Reaction<CountingReactor> {
    using Reaction<CountingReactor>::Reaction;
    Trigger<void> startup_trigger_{self().startup(), context()};
    void handler() final { self().startups_handled_++; }
  };

  void assemble() final { add_reaction<OnStartup>("on_startup"); }
};

} // namespace

TEST(runtime_provider, MismatchedProviderVersionIsRejected) {
  TestEnvironment env{};
  EXPECT_THROW(env.execute(MismatchedProvider{}), VersionMismatchError);
}

TEST(runtime_provider, ExplicitDefaultProviderExecutes) {
  TestEnvironment env{};
  CountingReactor reactor{"counter", env.context()};
  env.execute(DefaultRuntimeProvider{});
  EXPECT_EQ(reactor.startups_handled(), 1U);
}

} // namespace xronos::sdk::test
