// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_METRIC_HH
#define XRONOS_TELEMETRY_METRIC_HH

#include <cstdint>
#include <functional>
#include <variant>

#include "xronos/core/element.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::telemetry {

using MetricValue = std::variant<double, std::int64_t>;

class MetricDataLogger {
public:
  virtual ~MetricDataLogger() = default;

  virtual void record(const core::Element& metric, const runtime::TimeAccess& time_access, MetricValue value) = 0;
};

class NoopMetricDataLogger : public MetricDataLogger {
public:
  void record([[maybe_unused]] const core::Element& metric, [[maybe_unused]] const runtime::TimeAccess& time_access,
              [[maybe_unused]] MetricValue value) final {};
};

class MetricDataLoggerProvider {
public:
  auto logger() -> MetricDataLogger& { return logger_; }
  void set_logger(MetricDataLogger& logger) { logger_ = logger; }
  void reset_logger() { logger_ = default_logger_; }

private:
  NoopMetricDataLogger default_logger_{};
  std::reference_wrapper<MetricDataLogger> logger_{default_logger_};
};

} // namespace xronos::telemetry

#endif // XRONOS_TELEMETRY_METRIC_HH
