#include "flight_control/health/actuator_health_monitor.h"

HealthEvaluation ActuatorHealthMonitor::evaluate(const HealthContext &context)
{
  if (context.actuator_ready) return { HealthState::OK, HealthReason::NONE };
  return { context.armed ? HealthState::CRITICAL : HealthState::DEGRADED,
           HealthReason::ACTUATOR_FAILURE | HealthReason::NOT_READY };
}
