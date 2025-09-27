// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_TELEMETRY_HH
#define XRONOS_TELEMETRY_TELEMETRY_HH

#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/reaction.hh"

namespace xronos::telemetry {

class TelemetryBackend {
public:
  virtual ~TelemetryBackend() = default;
  virtual void initialize() = 0;
  virtual auto reaction_span_logger() -> ReactionSpanLogger& = 0;
  virtual auto metric_data_logger() -> MetricDataLogger& = 0;
};

class NoopTelemetryBackend : public TelemetryBackend {
public:
  void initialize() final {};
  auto reaction_span_logger() -> ReactionSpanLogger& final { return reaction_span_logger_; }
  auto metric_data_logger() -> MetricDataLogger& final { return metric_data_logger_; }

private:
  NoopReactionSpanLogger reaction_span_logger_{};
  NoopMetricDataLogger metric_data_logger_{};
};

} // namespace xronos::telemetry

#endif
