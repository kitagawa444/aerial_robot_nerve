#pragma once

#include "flight_control/health/health_monitor.h"

class SensorHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::SENSOR; }
  bool configure(const SensorHealthConfig &config);
  HealthEvaluation evaluate(const HealthContext &context) override;

private:
  SensorHealthConfig config_{};
};
