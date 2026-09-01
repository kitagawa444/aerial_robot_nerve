#include "flight_control/health/estimation_health_monitor.h"

HealthEvaluation EstimationHealthMonitor::evaluate(const HealthContext &context)
{
  if (context.estimation_ready) return { HealthState::OK, HealthReason::NONE };
  return { context.armed ? HealthState::CRITICAL : HealthState::DEGRADED,
           HealthReason::INVALID | HealthReason::NOT_READY };
}
