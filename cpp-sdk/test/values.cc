// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "xronos/sdk/value.hh"

namespace sdk = xronos::sdk;

namespace {

struct Message {
  std::string text;
};

// Views are scope-bound borrows: retaining one (by copy or move) must not
// compile. Owned values remain freely copyable.
static_assert(!std::is_copy_constructible_v<sdk::ValueView<Message>>);
static_assert(!std::is_move_constructible_v<sdk::ValueView<Message>>);
static_assert(!std::is_copy_assignable_v<sdk::ValueView<Message>>);
static_assert(!std::is_move_assignable_v<sdk::ValueView<Message>>);
static_assert(std::is_copy_constructible_v<sdk::Value<Message>>);

TEST(values, value_owns_and_shares) {
  auto value = sdk::Value<Message>{Message{"hello"}};
  ASSERT_TRUE(value);
  EXPECT_EQ(value->text, "hello");

  auto copy = value;
  EXPECT_EQ(copy.get(), value.get());
}

TEST(values, from_shared_ptr_shares_ownership_of_external_values) {
  auto external = std::make_shared<const Message>(Message{"external"});
  auto value = sdk::Value<Message>::from_shared_ptr(external);
  ASSERT_TRUE(value);
  EXPECT_EQ(value.get(), external.get());
  EXPECT_EQ(external.use_count(), 2);
}

TEST(values, from_unique_ptr_takes_over_ownership) {
  auto owned = std::make_unique<Message>(Message{"owned"});
  const auto* payload = owned.get();
  auto value = sdk::Value<Message>::from_unique_ptr(std::move(owned));
  ASSERT_TRUE(value);
  EXPECT_EQ(value.get(), payload);
}

TEST(values, from_unique_ptr_preserves_custom_deleters) {
  bool deleted = false;
  auto deleter = [&deleted](Message* message) {
    deleted = true;
    delete message; // NOLINT(cppcoreguidelines-owning-memory)
  };
  {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    std::unique_ptr<Message, decltype(deleter)> owned{new Message{"custom"}, deleter};
    auto value = sdk::Value<Message>::from_unique_ptr(std::move(owned));
    EXPECT_EQ(value->text, "custom");
    EXPECT_FALSE(deleted);
  }
  EXPECT_TRUE(deleted);
}

TEST(values, from_null_yields_empty_value) {
  auto value = sdk::Value<Message>::from_shared_ptr(nullptr);
  EXPECT_FALSE(value);
  EXPECT_EQ(value, nullptr);
}

TEST(values, as_shared_ptr_outlives_the_value) {
  auto value = sdk::Value<Message>{Message{"kept"}};
  auto pointer = value.as_shared_ptr();
  value = sdk::Value<Message>{};
  ASSERT_NE(pointer, nullptr);
  EXPECT_EQ(pointer->text, "kept");
}

TEST(values, as_shared_ptr_copies_inline_payloads) {
  auto value = sdk::Value<double>{1.5};
  auto first = value.as_shared_ptr();
  auto second = value.as_shared_ptr();
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(*first, 1.5);
  // Inline payloads are stored by value; pointer identity is not carried
  // through extraction.
  EXPECT_NE(first.get(), second.get());
}

TEST(values, empty_value_yields_null_shared_ptr) {
  const sdk::Value<Message> value{};
  EXPECT_EQ(value.as_shared_ptr(), nullptr);
}

} // namespace
