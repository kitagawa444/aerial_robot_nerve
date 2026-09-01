#pragma once

#include "flight_control/health/health_monitor.h"

class EstimationHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::ESTIMATION; }
  HealthEvaluation evaluate(const HealthContext &context) override;
};
