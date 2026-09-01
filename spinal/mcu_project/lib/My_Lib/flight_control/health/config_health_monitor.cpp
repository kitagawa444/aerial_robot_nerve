#include "flight_control/health/config_health_monitor.h"

HealthEvaluation ConfigHealthMonitor::evaluate(const HealthContext &context)
{
  if (context.config_ready) return { HealthState::OK, HealthReason::NONE };
  return { context.armed ? HealthState::CRITICAL : HealthState::DEGRADED,
           HealthReason::UNCONFIGURED | HealthReason::NOT_READY };
}
