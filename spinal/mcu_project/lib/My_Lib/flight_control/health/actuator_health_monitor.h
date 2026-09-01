#pragma once

#include "flight_control/health/health_monitor.h"

class ActuatorHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::ACTUATOR; }
  HealthEvaluation evaluate(const HealthContext &context) override;
};
