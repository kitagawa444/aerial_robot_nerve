#include "flight_control/health/flight_health_monitor.h"

HealthEvaluation FlightHealthMonitor::evaluate(const HealthContext &context)
{
  if (context.failsafe_active)
  {
    HealthEvaluation result{ HealthState::CRITICAL, HealthReason::FAILSAFE_ACTIVE };
    if (context.terminated) result.reason_mask |= HealthReason::TERMINATED;
    return result;
  }
  if (!context.ready_to_arm) return { HealthState::DEGRADED, HealthReason::NOT_READY };
  return { HealthState::OK, HealthReason::NONE };
}
