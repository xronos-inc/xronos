// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/telemetry/attribute_manager.hh"

using namespace xronos;

TEST_CASE("Single reactor", "[attribute manager]") {
  core::ElementRegistry element_registry{};
  telemetry::AttributeManager attribute_manager{};

  const auto& reactor = element_registry.add_new_element("reactor", core::ReactorTag{}, std::nullopt);

  SECTION("Attributes should be empty initially") {
    auto attributes = attribute_manager.get_attributes(reactor.uid, element_registry);
    REQUIRE(attributes.empty());
  }

  SECTION("Set some attributes individually") {
    attribute_manager.set_attribute(reactor.uid, "foo", 42);
    attribute_manager.set_attribute(reactor.uid, "bar", "bar");

    auto attributes = attribute_manager.get_attributes(reactor.uid, element_registry);
    REQUIRE(attributes.size() == 2);
    REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
    REQUIRE(std::get<std::string>(attributes["bar"]) == "bar");

    SECTION("Set more attributes and overwrite existing ones") {
      attribute_manager.set_attribute(reactor.uid, "bar", "barrrr");
      attribute_manager.set_attribute(reactor.uid, "baz", 42.0);

      auto attributes = attribute_manager.get_attributes(reactor.uid, element_registry);
      REQUIRE(attributes.size() == 3);
      REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
      REQUIRE(std::get<std::string>(attributes["bar"]) == "barrrr");
      REQUIRE(std::get<double>(attributes["baz"]) == 42.0);
    }
  }

  SECTION("Set some attributes using a dict") {
    attribute_manager.set_attributes(reactor.uid, {{"foo", 42}, {"bar", "bar"}});

    auto attributes = attribute_manager.get_attributes(reactor.uid, element_registry);
    REQUIRE(attributes.size() == 2);
    REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
    REQUIRE(std::get<std::string>(attributes["bar"]) == "bar");

    SECTION("Set more attributes and overwrite existing ones") {
      attribute_manager.set_attributes(reactor.uid, {{"bar", "barrrr"}, {"baz", 42.0}});

      auto attributes = attribute_manager.get_attributes(reactor.uid, element_registry);
      REQUIRE(attributes.size() == 3);
      REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
      REQUIRE(std::get<std::string>(attributes["bar"]) == "barrrr");
      REQUIRE(std::get<double>(attributes["baz"]) == 42.0);
    }
  }
}

TEST_CASE("Hierarchy", "[attribute manager]") {
  core::ElementRegistry element_registry{};
  telemetry::AttributeManager attribute_manager{};

  const auto& top = element_registry.add_new_element("top", core::ReactorTag{}, std::nullopt);
  const auto& nested1 = element_registry.add_new_element("nested1", core::ReactorTag{}, top.uid);
  const auto& nested2 = element_registry.add_new_element("nested2", core::ReactorTag{}, top.uid);
  const auto& timer1 = element_registry.add_new_element("timer", core::PeriodicTimerTag{}, nested1.uid);
  const auto& timer2 = element_registry.add_new_element("timer", core::PeriodicTimerTag{}, nested2.uid);
  const auto& reaction1 = element_registry.add_new_element("reaction", core::ReactionTag{}, nested1.uid);
  const auto& reaction2 = element_registry.add_new_element("reaction", core::ReactionTag{}, nested2.uid);

  std::vector<std::uint64_t> uids{top.uid,    nested1.uid,   nested2.uid,  timer1.uid,
                                  timer2.uid, reaction1.uid, reaction2.uid};

  SECTION("Attributes should be empty initially") {
    for (auto uid : uids) {
      REQUIRE(attribute_manager.get_attributes(uid, element_registry).empty());
    }
  }

  SECTION("All elements inherit attributes from the top level") {
    attribute_manager.set_attribute(top.uid, "foo", 42);
    for (auto uid : uids) {
      auto attributes = attribute_manager.get_attributes(uid, element_registry);
      REQUIRE(attributes.size() == 1);
      REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
    }

    SECTION("Nested elements may define additional attributes") {
      attribute_manager.set_attribute(nested1.uid, "bar", "one");
      attribute_manager.set_attribute(nested2.uid, "bar", "two");

      auto top_attributes = attribute_manager.get_attributes(top.uid, element_registry);
      REQUIRE(top_attributes.size() == 1);
      REQUIRE(std::get<std::int64_t>(top_attributes["foo"]) == 42);

      for (auto uid : {nested1.uid, timer1.uid, reaction1.uid}) {
        auto attributes = attribute_manager.get_attributes(uid, element_registry);
        REQUIRE(attributes.size() == 2);
        REQUIRE(std::get<std::string>(attributes["bar"]) == "one");
      }
      for (auto uid : {nested2.uid, timer2.uid, reaction2.uid}) {
        auto attributes = attribute_manager.get_attributes(uid, element_registry);
        REQUIRE(attributes.size() == 2);
        REQUIRE(std::get<std::string>(attributes["bar"]) == "two");
      }

      SECTION("Nested elements may overwrite their container's attributes") {
        attribute_manager.set_attribute(timer1.uid, "bar", "timer");
        attribute_manager.set_attribute(timer2.uid, "bar", "timer");
        attribute_manager.set_attribute(nested1.uid, "foo", 1);

        SECTION("Check that top is unmodified") {
          auto attributes = attribute_manager.get_attributes(top.uid, element_registry);
          REQUIRE(attributes.size() == 1);
          REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
        }
        SECTION("Check that nested1 uses overwritten value for 'foo'") {
          for (auto uid : {nested1.uid, timer1.uid, reaction1.uid}) {
            auto attributes = attribute_manager.get_attributes(uid, element_registry);
            REQUIRE(attributes.size() == 2);
            REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 1);
          }
        }
        SECTION("Check that nested2 is unmodified") {
          for (auto uid : {nested2.uid, timer2.uid, reaction2.uid}) {
            auto attributes = attribute_manager.get_attributes(uid, element_registry);
            REQUIRE(attributes.size() == 2);
            REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
          }
        }
        SECTION("Check that timers use overwritten value for 'bar'") {
          for (auto uid : {timer1.uid, timer2.uid}) {
            auto attributes = attribute_manager.get_attributes(uid, element_registry);
            REQUIRE(std::get<std::string>(attributes["bar"]) == "timer");
          }
        }
        SECTION("Check that other elements are not modified") {
          for (auto uid : {nested1.uid, reaction1.uid, nested2.uid, reaction2.uid}) {
            auto attributes = attribute_manager.get_attributes(uid, element_registry);
            REQUIRE(std::get<std::string>(attributes["bar"]) != "timer");
          }
        }
      }
    }
  }
}
