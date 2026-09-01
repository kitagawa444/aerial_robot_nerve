#pragma once

#include "flight_control/health/health_monitor.h"

class FlightHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::FLIGHT; }
  HealthEvaluation evaluate(const HealthContext &context) override;
};
