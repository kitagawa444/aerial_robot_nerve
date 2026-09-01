#pragma once

#include "flight_control/health/health_monitor.h"

class ComputeHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::COMPUTE; }
  bool configure(const ComputeHealthConfig &config);
  HealthEvaluation evaluate(const HealthContext &context) override;

private:
  ComputeHealthConfig config_{};
  uint8_t consecutive_misses_{ 0U };
  uint32_t last_sequence_{ 0U };
  HealthEvaluation last_evaluation_{};
};
