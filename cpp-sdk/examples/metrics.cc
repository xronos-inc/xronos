// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono> // IWYU pragma: keep
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

#include "xronos/sdk.hh"

namespace sdk = xronos::sdk;
using namespace std::literals::chrono_literals;

class Ramp : public sdk::Reactor {
public:
  Ramp(std::string_view name, const sdk::Context& context)
      : sdk::Reactor{name, context} {
    value_.add_attribute("value_category", "result");
  }
  [[nodiscard]] auto output() noexcept -> sdk::OutputPort<std::int64_t>& { return output_; }

private:
  sdk::PeriodicTimer timer_{"timer", context(), 100ms};
  sdk::OutputPort<std::int64_t> output_{"output", context()};
  sdk::Metric value_{"current_value", context(), "Current value of the Ramp data source."};

  std::int64_t count_{0};

  class OnTimerReaction : public sdk::Reaction<Ramp> {
    using sdk::Reaction<Ramp>::Reaction;

    Trigger<void> timer_trigger{self().timer_, context()};
    PortEffect<std::int64_t> output_effect{self().output_, context()};
    MetricEffect value_effect{self().value_, context()};

    void handler() final {
      value_effect.record(self().count_);
      output_effect.set(self().count_);
      self().count_++;
    }
  };

  void assemble() final { add_reaction<OnTimerReaction>("on_timer"); }
};

class Square : public sdk::Reactor {
public:
  Square(std::string_view name, const sdk::Context& context)
      : sdk::Reactor{name, context} {
    input_value_.add_attribute("value_category", "input");
    result_.add_attribute("value_category", "result");
  }

  using sdk::Reactor::Reactor;
  [[nodiscard]] auto input() noexcept -> sdk::InputPort<std::int64_t>& { return input_; }
  [[nodiscard]] auto output() noexcept -> sdk::OutputPort<std::int64_t>& { return output_; }

private:
  sdk::InputPort<std::int64_t> input_{"input", context()};
  sdk::OutputPort<std::int64_t> output_{"output", context()};
  sdk::Metric input_value_{"input_value", context(), "Last value received"};
  sdk::Metric result_{"result", context(), "input_value squared"};

  class OnInputReaction : public sdk::Reaction<Square> {
    using sdk::Reaction<Square>::Reaction;

    Trigger<std::int64_t> input_trigger{self().input_, context()};
    PortEffect<std::int64_t> output_effect{self().output_, context()};
    MetricEffect input_value_effect{self().input_value_, context()};
    MetricEffect result_effect{self().result_, context()};

    void handler() final {
      auto value = *input_trigger.get();
      input_value_effect.record(value);

      auto squared = value * value;
      result_effect.record(squared);

      output_effect.set(squared);
    }
  };

  void assemble() final { add_reaction<OnInputReaction>("on_input"); }
};

auto main() -> int {
  sdk::Environment env{};
  env.enable_telemetry();

  Ramp ramp{"ramp", env.context()};
  ramp.add_attribute("reactor_category", "data source");
  Square square{"square", env.context()};
  square.add_attributes({{"reactor_category", "data processor"}, {"operation", "square"}});

  std::unordered_map<std::string, sdk::AttributeValue> map{};
  square.add_attributes(map);

  env.connect(ramp.output(), square.input(), 1s);

  env.execute();

  return 0;
}
