#include "flight_control/health/sensor_health_monitor.h"

bool SensorHealthMonitor::configure(const SensorHealthConfig &config)
{
  if (config.primary_imu_timeout_ms == 0U) return false;
  config_ = config;
  return true;
}

HealthEvaluation SensorHealthMonitor::evaluate(const HealthContext &context)
{
  if (!context.primary_imu_present) return { HealthState::UNKNOWN, HealthReason::DATA_UNAVAILABLE };
  if (context.last_primary_imu_update_ms == 0U) return { HealthState::DEGRADED, HealthReason::NOT_READY };

  const uint32_t age_ms = context.now_ms - context.last_primary_imu_update_ms;
  if (age_ms > config_.primary_imu_timeout_ms)
  {
    return { HealthState::CRITICAL, HealthReason::SENSOR_FAILURE | HealthReason::STALE };
  }
  return { HealthState::OK, HealthReason::NONE };
}
