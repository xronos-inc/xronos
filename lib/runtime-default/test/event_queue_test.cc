// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// Pins EventQueue's wait under a fake clock: the next tag's events come back
// once the clock reaches it, a physical event or a stop request ends a wait in
// progress, and fast mode without physical events never waits.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <thread>
#include <vector>

#include "xronos/abi/types.hh"
#include "xronos/abi/value.hh"
#include "xronos/core/time.hh"
#include "xronos/logical_time/tag.hh"
#include "xronos/physical_time/testing/fake_clock.hh"
#include "xronos/runtime/default/detail/scheduler.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/value/boxing.hh"
#include "gtest/gtest.h"

namespace {

using namespace std::chrono_literals;
using xronos::physical_time::FakeClock;
using xronos::runtime::ExecutionProperties;
using xronos::runtime::default_::detail::Event;
using xronos::runtime::default_::detail::EventQueue;
using Events = std::map<xronos::logical_time::Tag, std::vector<Event>>::node_type;

auto tag_at(std::chrono::seconds seconds) -> xronos::logical_time::Tag {
  return xronos::logical_time::Tag{xronos::core::TimePoint{} + seconds, 0};
}

auto payload() -> xronos::abi::AnyValue { return xronos::value::make<int>(0); }

TEST(EventQueueTest, TheWaitReturnsTheHeadOnceTheClockReachesItsTag) {
  FakeClock clock{};
  EventQueue queue{ExecutionProperties{}, clock};
  queue.schedule_event(1, payload(), tag_at(10s));

  std::atomic<bool> returned{false};
  Events events{};
  std::jthread waiter{[&]() {
    events = queue.wait_until_next_event(xronos::logical_time::Tag::max());
    returned = true;
  }};

  clock.set_now(tag_at(5s).timestamp());
  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(returned);

  clock.set_now(tag_at(10s).timestamp());
  waiter.join();

  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.key(), tag_at(10s));
  EXPECT_EQ(events.mapped().front().element_uid, 1U);
}

TEST(EventQueueTest, APhysicalEventEndsTheWaitAndComesBackAsTheHead) {
  FakeClock clock{tag_at(5s).timestamp()};
  EventQueue queue{ExecutionProperties{}, clock};
  queue.set_has_physical_events();
  queue.mark_started(xronos::logical_time::Tag::max());
  queue.schedule_event(1, payload(), tag_at(10s));

  Events events{};
  std::jthread waiter{[&]() { events = queue.wait_until_next_event(xronos::logical_time::Tag::max()); }};

  std::this_thread::sleep_for(10ms);
  auto status = queue.trigger_physical_event(2, payload());
  EXPECT_EQ(status, xronos::abi::TriggerStatus::Accepted);
  if (status != xronos::abi::TriggerStatus::Accepted) {
    // Nothing else would end the wait, and the join below would hang.
    queue.request_stop();
  }
  waiter.join();

  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.key(), tag_at(5s));
  EXPECT_EQ(events.mapped().front().element_uid, 2U);
}

TEST(EventQueueTest, AStopRequestEndsTheWaitWithAnEmptyResult) {
  FakeClock clock{};
  EventQueue queue{ExecutionProperties{}, clock};
  queue.schedule_event(1, payload(), tag_at(10s));

  Events events{};
  std::jthread waiter{[&]() { events = queue.wait_until_next_event(xronos::logical_time::Tag::max()); }};

  std::this_thread::sleep_for(10ms);
  queue.request_stop();
  waiter.join();

  EXPECT_TRUE(events.empty());
}

TEST(EventQueueTest, WithPhysicalEventsAnEmptyQueueWaitsForMaxTag) {
  FakeClock clock{};
  EventQueue queue{ExecutionProperties{}, clock};
  queue.set_has_physical_events();

  std::atomic<bool> returned{false};
  Events events{};
  std::jthread waiter{[&]() {
    events = queue.wait_until_next_event(tag_at(10s));
    returned = true;
  }};

  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(returned);

  clock.set_now(tag_at(10s).timestamp());
  waiter.join();

  EXPECT_TRUE(events.empty());
}

TEST(EventQueueTest, FastModeWithoutPhysicalEventsReturnsWithoutWaiting) {
  const FakeClock clock{};
  EventQueue queue{ExecutionProperties{.fast_mode = true}, clock};
  queue.schedule_event(1, payload(), tag_at(10s));

  auto events = queue.wait_until_next_event(xronos::logical_time::Tag::max());

  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.key(), tag_at(10s));
}

} // namespace
