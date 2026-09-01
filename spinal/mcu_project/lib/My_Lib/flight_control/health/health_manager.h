#pragma once

#include <cstddef>

#include "flight_control/health/actuator_health_monitor.h"
#include "flight_control/health/compute_health_monitor.h"
#include "flight_control/health/config_health_monitor.h"
#include "flight_control/health/estimation_health_monitor.h"
#include "flight_control/health/flight_health_monitor.h"
#include "flight_control/health/link_health_monitor.h"
#include "flight_control/health/power_health_monitor.h"
#include "flight_control/health/sensor_health_monitor.h"

// HealthManager owns a fixed registry of allocation-free monitors and is the
// sole writer of component timestamps and overall health.
class HealthManager
{
public:
  HealthManager();
  HealthManager(const HealthManager &) = delete;
  HealthManager &operator=(const HealthManager &) = delete;

  bool configure(const HealthManagerConfig &config);
  bool update(const HealthContext &context, FlightHealthReport &report);

private:
  static constexpr std::size_t MONITOR_COUNT = 8U;

  FlightHealthMonitor flight_monitor_{};
  LinkHealthMonitor link_monitor_{};
  PowerHealthMonitor power_monitor_{};
  SensorHealthMonitor sensor_monitor_{};
  EstimationHealthMonitor estimation_monitor_{};
  ActuatorHealthMonitor actuator_monitor_{};
  ComputeHealthMonitor compute_monitor_{};
  ConfigHealthMonitor config_monitor_{};
  HealthMonitor *monitors_[MONITOR_COUNT];

  static HealthComponentStatus &componentStatus_(FlightHealthReport &report, HealthComponent component);
  static bool updateStatus_(HealthComponentStatus &status, const HealthEvaluation &evaluation, uint32_t now_ms);
  static HealthEvaluation aggregate_(const FlightHealthReport &report);
  static bool selectAction_(const HealthContext &context, FlightHealthReport &report);
};
