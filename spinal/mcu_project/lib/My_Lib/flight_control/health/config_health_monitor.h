#pragma once

#include "flight_control/health/health_monitor.h"

class ConfigHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::CONFIG; }
  HealthEvaluation evaluate(const HealthContext &context) override;
};
