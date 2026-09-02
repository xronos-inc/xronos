// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <string>
#include <utility>
#include <vector>

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

namespace xronos::sdk::test {

namespace {

class EmptyReactor : public Reactor {
  using Reactor::Reactor;
  void assemble() final {}
};

} // namespace

TEST(attributes, AddAttributeRejectsDuplicateKey) {
  TestEnvironment env{};
  EmptyReactor reactor{"reactor", env.context()};

  EXPECT_TRUE(reactor.add_attribute("asil", std::string{"B"}));
  EXPECT_FALSE(reactor.add_attribute("asil", std::string{"D"}));
}

TEST(attributes, AddAttributesReportsDuplicateKey) {
  TestEnvironment env{};
  EmptyReactor reactor{"reactor", env.context()};

  ASSERT_TRUE(reactor.add_attribute("asil", std::string{"B"}));

  // The batch holds a duplicate of "asil", so it must report failure.
  EXPECT_FALSE(reactor.add_attributes({
      {"asil", std::string{"D"}},
      {"revision", 3.0},
  }));

  // Attributes apply per key: the non-duplicate "revision" was still added.
  EXPECT_FALSE(reactor.add_attribute("revision", 4.0));
}

TEST(attributes, AddAttributesRangeReportsDuplicateKey) {
  TestEnvironment env{};
  EmptyReactor reactor{"reactor", env.context()};

  const std::vector<std::pair<std::string, AttributeValue>> attributes{
      {"key", std::string{"value"}},
      {"key", std::string{"other"}},
  };
  EXPECT_FALSE(reactor.add_attributes(attributes));
}

TEST(attributes, AddAttributesSucceedsWithoutDuplicates) {
  TestEnvironment env{};
  EmptyReactor reactor{"reactor", env.context()};

  EXPECT_TRUE(reactor.add_attributes({
      {"asil", std::string{"B"}},
      {"revision", 3.0},
  }));
}

} // namespace xronos::sdk::test
