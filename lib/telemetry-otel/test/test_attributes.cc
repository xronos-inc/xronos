// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <functional>
#include <string_view>
#include <vector>

#include "common.hh"
#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/runtime/reactor.hh"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/telemetry/attribute_manager.hh"

using namespace xronos::runtime;
using namespace xronos::telemetry;
using namespace xronos::telemetry::otel;

class TestReactor : public Reactor {
public:
  TestReactor(const std::string& name, Environment& environment)
      : Reactor(name, environment) {}
  TestReactor(const std::string& name, Reactor& container)
      : Reactor(name, container) {}

  void assemble() final {}
};

TEST_CASE("Single reactor", "[attribute manager]") {
  Environment env{1};
  TestReactor reactor{"reactor", env};
  AttributeManager attribute_manager{};

  SECTION("Attributes should be empty initially") {
    auto attributes = get_merged_attributes(attribute_manager, reactor);
    REQUIRE(attributes.empty());
  }

  SECTION("Set some attributes individually") {
    attribute_manager.set_attribute(reactor, "foo", 42);
    attribute_manager.set_attribute(reactor, "bar", "bar");

    auto attributes = get_merged_attributes(attribute_manager, reactor);
    REQUIRE(attributes.size() == 2);
    REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
    auto expected = OtelAttributeValue("bar");
    REQUIRE(std::get<std::string_view>(attributes["bar"]) == "bar");

    SECTION("Set more attributes and overwrite existing ones") {
      attribute_manager.set_attribute(reactor, "bar", "barrrr");
      attribute_manager.set_attribute(reactor, "baz", 42.0);

      auto attributes = get_merged_attributes(attribute_manager, reactor);
      REQUIRE(attributes.size() == 3);
      REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
      REQUIRE(std::get<std::string_view>(attributes["bar"]) == "barrrr");
      REQUIRE(std::get<double>(attributes["baz"]) == 42.0);
    }
  }

  SECTION("Set some attributes using a dict") {
    attribute_manager.set_attributes(reactor, {{"foo", 42}, {"bar", "bar"}});

    auto attributes = get_merged_attributes(attribute_manager, reactor);
    REQUIRE(attributes.size() == 2);
    REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
    REQUIRE(std::get<std::string_view>(attributes["bar"]) == "bar");

    SECTION("Set more attributes and overwrite existing ones") {
      attribute_manager.set_attributes(reactor, {{"bar", "barrrr"}, {"baz", 42.0}});

      auto attributes = get_merged_attributes(attribute_manager, reactor);
      REQUIRE(attributes.size() == 3);
      REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
      REQUIRE(std::get<std::string_view>(attributes["bar"]) == "barrrr");
      REQUIRE(std::get<double>(attributes["baz"]) == 42.0);
    }
  }
}

TEST_CASE("Hierarchy", "[attribute manager]") {
  AttributeManager attribute_manager{};
  Environment env{1};
  TestReactor top{"top", env};
  TestReactor nested1{"nested1", top};
  TestReactor nested2{"nested2", top};
  Timer timer1{"timer", nested1};
  Timer timer2{"timer", nested2};
  Reaction reaction1{"reaction", 1, nested1, []() {}};
  Reaction reaction2{"reaction", 1, nested2, []() {}};

  auto elements =
      std::vector<std::reference_wrapper<ReactorElement>>{top, nested1, nested2, timer1, timer2, reaction1, reaction2};

  SECTION("Attributes should be empty initially") {
    for (auto elem : elements) {
      REQUIRE(get_merged_attributes(attribute_manager, elem).empty());
    }
  }

  SECTION("All elements inherit attributes from the top level") {
    attribute_manager.set_attribute(top, "foo", 42);
    for (auto elem : elements) {
      auto attributes = get_merged_attributes(attribute_manager, elem);
      REQUIRE(attributes.size() == 1);
      REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
    }

    SECTION("Nested elements may define additional attributes") {
      attribute_manager.set_attribute(nested1, "bar", "one");
      attribute_manager.set_attribute(nested2, "bar", "two");

      auto top_attributes = get_merged_attributes(attribute_manager, top);
      REQUIRE(top_attributes.size() == 1);
      REQUIRE(std::get<std::int64_t>(top_attributes["foo"]) == 42);

      for (auto elem : std::vector<std::reference_wrapper<ReactorElement>>{nested1, timer1, reaction1}) {
        auto attributes = get_merged_attributes(attribute_manager, elem);
        REQUIRE(attributes.size() == 2);
        REQUIRE(std::get<std::string_view>(attributes["bar"]) == "one");
      }
      for (auto elem : std::vector<std::reference_wrapper<ReactorElement>>{nested2, timer2, reaction2}) {
        auto attributes = get_merged_attributes(attribute_manager, elem);
        REQUIRE(attributes.size() == 2);
        REQUIRE(std::get<std::string_view>(attributes["bar"]) == "two");
      }

      SECTION("Nested elements may overwrite their container's attributes") {
        attribute_manager.set_attribute(timer1, "bar", "timer");
        attribute_manager.set_attribute(timer2, "bar", "timer");
        attribute_manager.set_attribute(nested1, "foo", 1);

        SECTION("Check that top is unmodified") {
          auto attributes = get_merged_attributes(attribute_manager, top);
          REQUIRE(attributes.size() == 1);
          REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
        }
        SECTION("Check that nested1 uses overwritten value for 'foo'") {
          for (auto elem : std::vector<std::reference_wrapper<ReactorElement>>{nested1, timer1, reaction1}) {
            auto attributes = get_merged_attributes(attribute_manager, elem);
            REQUIRE(attributes.size() == 2);
            REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 1);
          }
        }
        SECTION("Check that nested2 is unmodified") {
          for (auto elem : std::vector<std::reference_wrapper<ReactorElement>>{nested2, timer2, reaction2}) {
            auto attributes = get_merged_attributes(attribute_manager, elem);
            REQUIRE(attributes.size() == 2);
            REQUIRE(std::get<std::int64_t>(attributes["foo"]) == 42);
          }
        }
        SECTION("Check that timers use overwritten value for 'bar'") {
          for (auto elem : std::vector<std::reference_wrapper<ReactorElement>>{timer1, timer2}) {
            auto attributes = get_merged_attributes(attribute_manager, elem);
            REQUIRE(std::get<std::string_view>(attributes["bar"]) == "timer");
          }
        }
        SECTION("Check that other elements are not modified'") {
          for (auto elem :
               std::vector<std::reference_wrapper<ReactorElement>>{nested1, nested2, reaction1, reaction2}) {
            auto attributes = get_merged_attributes(attribute_manager, elem);
            REQUIRE(std::get<std::string_view>(attributes["bar"]) != "timer");
          }
        }
      }
    }
  }
}
