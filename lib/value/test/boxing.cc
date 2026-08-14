// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "xronos/abi/value.hh"
#include "xronos/value/boxing.hh"

namespace {

using xronos::abi::AnyValue;
using xronos::value::as_shared_ptr;
using xronos::value::from_shared_ptr;
using xronos::value::get_if;
using xronos::value::holds;
using xronos::value::make;

// 24 bytes, trivially copyable: the largest payload that still fits inline.
struct Vec3 {
  double x;
  double y;
  double z;
};
static_assert(xronos::value::detail::uses_inline_value_storage<Vec3>);

// 32 bytes, trivially copyable: still within the inline capacity.
struct Quaternion {
  double w;
  double x;
  double y;
  double z;
};
static_assert(xronos::value::detail::uses_inline_value_storage<Quaternion>);

// 56 bytes, trivially copyable: exactly fills the inline buffer.
struct Pose {
  Vec3 position;
  Quaternion orientation;
};
static_assert(xronos::value::detail::uses_inline_value_storage<Pose>);

// 72 bytes, trivially copyable: exceeds the inline capacity.
struct Matrix3 {
  std::array<double, 9> values;
};
static_assert(!xronos::value::detail::uses_inline_value_storage<Matrix3>);

// Small and trivially copyable, but over-aligned: must not be stored inline.
struct alignas(16) OverAligned {
  double value;
};
static_assert(!xronos::value::detail::uses_inline_value_storage<OverAligned>);

// Non-trivial payload that counts live instances and copies.
struct Tracked {
  static inline std::atomic<int> live{0};
  static inline std::atomic<int> copies{0};

  explicit Tracked(std::string text)
      : text{std::move(text)} {
    live++;
  }
  Tracked(const Tracked& other)
      : text{other.text} {
    live++;
    copies++;
  }
  Tracked(Tracked&&) = delete;
  auto operator=(const Tracked&) -> Tracked& = delete;
  auto operator=(Tracked&&) -> Tracked& = delete;
  ~Tracked() { live--; }

  std::string text;
};
static_assert(!xronos::value::detail::uses_inline_value_storage<Tracked>);

// True if `pointer` points into the object representation of `value`.
auto points_into(const void* pointer, const AnyValue& value) -> bool {
  const auto* begin = reinterpret_cast<const std::byte*>(&value);
  const auto* target = static_cast<const std::byte*>(pointer);
  return target >= begin && target < begin + sizeof(AnyValue);
}

TEST_CASE("default-constructed AnyValue is empty", "[value][boxing]") {
  const AnyValue value{};
  CHECK_FALSE(value.has_value());
  CHECK(value.manager() == nullptr);
  CHECK(value.payload() == nullptr);
  CHECK(get_if<int>(value) == nullptr);
  CHECK_FALSE(holds<int>(value));
}

TEST_CASE("empty AnyValue can be copied, moved, and reset", "[value][boxing]") {
  AnyValue value{};
  AnyValue copy{value}; // NOLINT(performance-unnecessary-copy-initialization)
  const AnyValue moved{std::move(value)};
  CHECK_FALSE(copy.has_value());
  CHECK_FALSE(moved.has_value());
  copy.reset();
  CHECK_FALSE(copy.has_value());
}

TEST_CASE("small trivially copyable payloads are stored inline", "[value][boxing]") {
  const auto value = make<std::int64_t>(42);
  REQUIRE(value.has_value());
  REQUIRE(holds<std::int64_t>(value));
  CHECK_FALSE(holds<double>(value));
  CHECK(get_if<double>(value) == nullptr);
  REQUIRE(get_if<std::int64_t>(value) != nullptr);
  CHECK(*get_if<std::int64_t>(value) == 42);
  CHECK(points_into(value.payload(), value));
}

TEST_CASE("inline payloads have value semantics", "[value][boxing]") {
  const auto value = make<Vec3>(1.0, 2.0, 3.0);
  const AnyValue copy{value}; // NOLINT(performance-unnecessary-copy-initialization)
  REQUIRE(holds<Vec3>(copy));
  CHECK(get_if<Vec3>(copy)->y == 2.0);
  // Each copy owns its own inline payload.
  CHECK(value.payload() != copy.payload());
  CHECK(points_into(copy.payload(), copy));
}

TEST_CASE("large trivially copyable payloads use shared heap storage", "[value][boxing]") {
  const auto value = make<Matrix3>(std::array<double, 9>{1.0});
  const AnyValue copy{value}; // NOLINT(performance-unnecessary-copy-initialization)
  REQUIRE(holds<Matrix3>(copy));
  CHECK(get_if<Matrix3>(copy)->values[0] == 1.0);
  // Copies share one payload; nothing points into the 64-byte value itself.
  CHECK(value.payload() == copy.payload());
  CHECK_FALSE(points_into(value.payload(), value));
}

TEST_CASE("over-aligned payloads use heap storage and are properly aligned", "[value][boxing]") {
  const auto value = make<OverAligned>(1.5);
  REQUIRE(holds<OverAligned>(value));
  CHECK(reinterpret_cast<std::uintptr_t>(value.payload()) % alignof(OverAligned) == 0);
  CHECK(get_if<OverAligned>(value)->value == 1.5);
}

TEST_CASE("copying shares the payload instead of copying it", "[value][boxing]") {
  Tracked::live = 0;
  Tracked::copies = 0;
  {
    auto value = make<Tracked>("hello");
    CHECK(Tracked::live == 1);
    {
      const AnyValue copy{value};   // NOLINT(performance-unnecessary-copy-initialization)
      const AnyValue another{copy}; // NOLINT(performance-unnecessary-copy-initialization)
      CHECK(Tracked::live == 1);
      CHECK(Tracked::copies == 0);
      CHECK(get_if<Tracked>(copy) == get_if<Tracked>(value));
      CHECK(get_if<Tracked>(another)->text == "hello");
      // Dropping one reference keeps the payload alive.
      value.reset();
      CHECK(Tracked::live == 1);
      CHECK(get_if<Tracked>(another)->text == "hello");
    }
    CHECK(Tracked::live == 0);
  }
}

TEST_CASE("moving transfers ownership without touching the payload", "[value][boxing]") {
  Tracked::live = 0;
  Tracked::copies = 0;
  auto value = make<Tracked>("moved");
  const auto* payload_before = get_if<Tracked>(value);
  const AnyValue target{std::move(value)};
  CHECK_FALSE(value.has_value()); // NOLINT(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  CHECK(Tracked::live == 1);
  CHECK(Tracked::copies == 0);
  CHECK(get_if<Tracked>(target) == payload_before);
}

TEST_CASE("assignment replaces the previous payload", "[value][boxing]") {
  Tracked::live = 0;
  auto value = make<Tracked>("first");
  value = make<Tracked>("second");
  CHECK(Tracked::live == 1);
  CHECK(get_if<Tracked>(value)->text == "second");

  AnyValue copy{};
  copy = value; // copy assignment
  CHECK(Tracked::live == 1);
  CHECK(get_if<Tracked>(copy) == get_if<Tracked>(value));

  value = AnyValue{}; // assigning empty drops the reference
  CHECK(Tracked::live == 1);
  copy.reset();
  CHECK(Tracked::live == 0);
}

TEST_CASE("self-assignment is a no-op", "[value][boxing]") {
  Tracked::live = 0;
  auto value = make<Tracked>("self");
  auto& reference = value;
  value = reference;
  CHECK(Tracked::live == 1);
  CHECK(get_if<Tracked>(value)->text == "self");
  value = std::move(reference);
  CHECK(Tracked::live == 1);
  CHECK(get_if<Tracked>(value)->text == "self"); // NOLINT(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
}

TEST_CASE("from_shared_ptr shares ownership without copying the payload", "[value][boxing]") {
  auto external = std::make_shared<const Matrix3>(Matrix3{{1.0}});
  const auto value = from_shared_ptr<Matrix3>(external);
  REQUIRE(holds<Matrix3>(value));
  // The stored payload IS the external object: identity is preserved.
  CHECK(get_if<Matrix3>(value) == external.get());
  CHECK(external.use_count() == 2);

  auto extracted = as_shared_ptr<Matrix3>(value);
  CHECK(extracted.get() == external.get());
  CHECK(external.use_count() == 3);
}

TEST_CASE("from_shared_ptr copies payloads that are stored inline", "[value][boxing]") {
  auto external = std::make_shared<const Vec3>(Vec3{1.0, 2.0, 3.0});
  const auto value = from_shared_ptr<Vec3>(external);
  REQUIRE(holds<Vec3>(value));
  // Inline payloads are copied; no ownership relation to the external pointer.
  CHECK(get_if<Vec3>(value) != external.get());
  CHECK(get_if<Vec3>(value)->y == 2.0);
  CHECK(external.use_count() == 1);
  CHECK(points_into(value.payload(), value));

  // Extraction allocates a fresh copy on every call.
  auto first = as_shared_ptr<Vec3>(value);
  auto second = as_shared_ptr<Vec3>(value);
  CHECK(first->z == 3.0);
  CHECK(first.get() != second.get());
}

TEST_CASE("from_shared_ptr with a null pointer yields the empty value", "[value][boxing]") {
  const auto value = from_shared_ptr<Tracked>(nullptr);
  CHECK_FALSE(value.has_value());
}

TEST_CASE("from_shared_ptr accepts a unique_ptr", "[value][boxing]") {
  Tracked::live = 0;
  Tracked::copies = 0;
  auto owned = std::make_unique<Tracked>("adopted");
  const auto* payload_before = owned.get();
  const auto value = from_shared_ptr<Tracked>(std::move(owned));
  CHECK(Tracked::live == 1);
  CHECK(Tracked::copies == 0);
  CHECK(get_if<Tracked>(value) == payload_before);
}

TEST_CASE("as_shared_ptr keeps the payload alive beyond the value", "[value][boxing]") {
  Tracked::live = 0;
  auto value = make<Tracked>("outlives");
  auto pointer = as_shared_ptr<Tracked>(value);
  value.reset();
  CHECK(Tracked::live == 1);
  CHECK(pointer->text == "outlives");
  pointer.reset();
  CHECK(Tracked::live == 0);
}

TEST_CASE("as_shared_ptr returns null for empty values and type mismatches", "[value][boxing]") {
  const AnyValue empty{};
  CHECK(as_shared_ptr<Tracked>(empty) == nullptr);
  const auto value = make<std::int64_t>(7);
  CHECK(as_shared_ptr<Tracked>(value) == nullptr);
}

TEST_CASE("distinct payload types do not alias", "[value][boxing]") {
  struct A {
    std::int32_t value;
  };
  struct B {
    std::int32_t value;
  };
  const auto value = make<A>(7);
  CHECK(holds<A>(value));
  CHECK_FALSE(holds<B>(value));
  CHECK(get_if<B>(value) == nullptr);
}

} // namespace
