#pragma once

#include "flight_control/health/health_monitor.h"

class LinkHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::LINK; }
  HealthEvaluation evaluate(const HealthContext &context) override;
};
