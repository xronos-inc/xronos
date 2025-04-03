// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_METRIC_HH
#define XRONOS_TELEMETRY_METRIC_HH

#include "xronos/runtime/misc_element.hh"
#include "xronos/runtime/reactor.hh"
#include <cstdint>
#include <variant>

namespace xronos::telemetry {

class Metric;

using MetricValue = std::variant<double, std::int64_t>;

class MetricDataLogger {
public:
  virtual ~MetricDataLogger() = default;

  virtual void record(const Metric& metric, MetricValue value) = 0;
};

class NoopMetricDataLogger : public MetricDataLogger {
public:
  void record([[maybe_unused]] const Metric& metric, [[maybe_unused]] MetricValue value) final {};
};

class MetricDataLoggerProvider {
private:
  NoopMetricDataLogger default_logger_{};
  std::reference_wrapper<MetricDataLogger> logger_{default_logger_};

public:
  auto logger() -> MetricDataLogger& { return logger_; }
  void set_logger(MetricDataLogger& logger) { logger_ = logger; }
  void reset_logger() { logger_ = default_logger_; }
};

class Metric : public runtime::MiscElement {
private:
  std::reference_wrapper<MetricDataLoggerProvider> logger_provider_;
  std::string description_;
  std::string unit_;

public:
  Metric(std::string_view name, runtime::Reactor& container, MetricDataLoggerProvider& logger_provider,
         std::string_view description, std::string_view unit)
      : MiscElement(name, container)
      , logger_provider_(logger_provider)
      , description_(description)
      , unit_(unit) {}

  [[nodiscard]] auto description() const -> const std::string& { return description_; }

  [[nodiscard]] auto unit() const -> const std::string& { return unit_; }

  void record(MetricValue value) { logger_provider_.get().logger().record(*this, value); }

  [[nodiscard]] auto element_type() const -> std::string_view final { return "metric"; }
};

} // namespace xronos::telemetry

#endif // XRONOS_TELEMETRY_METRIC_HH
