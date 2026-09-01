#pragma once

#include "flight_control/health/health_types.h"

// A monitor is a statically registered FC plugin. Implementations are kept
// allocation-free and only evaluate the supplied immutable context.
class HealthMonitor
{
public:
  virtual ~HealthMonitor() = default;

  virtual HealthComponent component() const = 0;
  virtual HealthEvaluation evaluate(const HealthContext &context) = 0;
};
