#include "flight_control/health/health_manager.h"

HealthManager::HealthManager()
  : monitors_{ &flight_monitor_,     &link_monitor_,     &power_monitor_,   &sensor_monitor_,
               &estimation_monitor_, &actuator_monitor_, &compute_monitor_, &config_monitor_ }
{
}

bool HealthManager::configure(const HealthManagerConfig &config)
{
  const bool power_configured = power_monitor_.configure(config.power);
  const bool sensor_configured = sensor_monitor_.configure(config.sensor);
  const bool compute_configured = compute_monitor_.configure(config.compute);
  return power_configured && sensor_configured && compute_configured;
}

bool HealthManager::update(const HealthContext &context, FlightHealthReport &report)
{
  bool changed = report.link.ros != context.ros_link_state || report.link.network != context.network_link_state ||
                 report.link.rc != context.rc_link_state || report.estimation_validity != context.estimation_validity;

  report.link.ros = context.ros_link_state;
  report.link.network = context.network_link_state;
  report.link.rc = context.rc_link_state;
  report.battery_voltage = context.battery_voltage;
  report.estimation_validity = context.estimation_validity;
  report.rejected_measurements = context.rejected_measurements;

  for (HealthMonitor *monitor : monitors_)
  {
    HealthComponentStatus &status = componentStatus_(report, monitor->component());
    changed |= updateStatus_(status, monitor->evaluate(context), context.now_ms);
  }

  report.battery_compensated_voltage = power_monitor_.compensatedVoltage();
  report.battery_percentage = power_monitor_.batteryPercentage();
  changed |= updateStatus_(report.overall, aggregate_(report), context.now_ms);
  changed |= selectAction_(context, report);
  return changed;
}

HealthComponentStatus &HealthManager::componentStatus_(FlightHealthReport &report, HealthComponent component)
{
  switch (component)
  {
    case HealthComponent::FLIGHT:
      return report.flight;
    case HealthComponent::LINK:
      return report.link.summary;
    case HealthComponent::POWER:
      return report.power;
    case HealthComponent::SENSOR:
      return report.sensor;
    case HealthComponent::ESTIMATION:
      return report.estimation;
    case HealthComponent::ACTUATOR:
      return report.actuator;
    case HealthComponent::COMPUTE:
      return report.compute;
    case HealthComponent::CONFIG:
      return report.config;
  }
  return report.overall;
}

bool HealthManager::updateStatus_(HealthComponentStatus &status, const HealthEvaluation &evaluation, uint32_t now_ms)
{
  if (status.state == evaluation.state && status.reason_mask == evaluation.reason_mask) return false;
  status.state = evaluation.state;
  status.reason_mask = evaluation.reason_mask;
  status.last_change_ms = now_ms;
  return true;
}

HealthEvaluation HealthManager::aggregate_(const FlightHealthReport &report)
{
  const HealthComponentStatus *required[] = { &report.flight, &report.estimation, &report.actuator, &report.config };
  const HealthComponentStatus *optional[] = { &report.power, &report.sensor, &report.compute };
  uint32_t reasons = HealthReason::NONE;

  for (const HealthComponentStatus *component : required)
  {
    reasons |= component->reason_mask;
  }
  for (const HealthComponentStatus *component : optional)
  {
    if (component->state == HealthState::DEGRADED || component->state == HealthState::CRITICAL)
      reasons |= component->reason_mask;
  }
  if (report.link.summary.state == HealthState::CRITICAL || report.link.summary.state == HealthState::DEGRADED)
    reasons |= report.link.summary.reason_mask;

  for (const HealthComponentStatus *component : required)
  {
    if (component->state == HealthState::CRITICAL) return { HealthState::CRITICAL, reasons };
  }
  for (const HealthComponentStatus *component : optional)
  {
    if (component->state == HealthState::CRITICAL) return { HealthState::CRITICAL, reasons };
  }

  if (report.link.summary.state == HealthState::CRITICAL || report.link.summary.state == HealthState::DEGRADED)
    return { HealthState::DEGRADED, reasons };

  for (const HealthComponentStatus *component : required)
  {
    if (component->state == HealthState::DEGRADED) return { HealthState::DEGRADED, reasons };
  }
  for (const HealthComponentStatus *component : optional)
  {
    if (component->state == HealthState::DEGRADED) return { HealthState::DEGRADED, reasons };
  }

  const bool required_ready = report.flight.state == HealthState::OK && report.estimation.state == HealthState::OK &&
                              report.actuator.state == HealthState::OK && report.config.state == HealthState::OK;
  return required_ready ? HealthEvaluation{ HealthState::OK, HealthReason::NONE } :
                          HealthEvaluation{ HealthState::UNKNOWN, reasons };
}

bool HealthManager::selectAction_(const HealthContext &context, FlightHealthReport &report)
{
  uint8_t action = HealthAction::NONE;
  HealthComponent source = HealthComponent::FLIGHT;
  uint32_t reasons = HealthReason::NONE;

  const auto choose = [&action, &source, &reasons](uint8_t candidate, HealthComponent candidate_source,
                                                   const HealthComponentStatus &status)
  {
    if (candidate <= action) return;
    action = candidate;
    source = candidate_source;
    reasons = status.reason_mask;
  };

  if (!context.armed)
  {
    if (!context.ready_to_arm) choose(HealthAction::INHIBIT_ARM, HealthComponent::FLIGHT, report.flight);
    if (report.power.state == HealthState::CRITICAL || report.power.state == HealthState::DEGRADED)
      choose(HealthAction::INHIBIT_ARM, HealthComponent::POWER, report.power);
    if (report.sensor.state == HealthState::CRITICAL || report.sensor.state == HealthState::DEGRADED)
      choose(HealthAction::INHIBIT_ARM, HealthComponent::SENSOR, report.sensor);
    if (report.compute.state == HealthState::CRITICAL)
      choose(HealthAction::INHIBIT_ARM, HealthComponent::COMPUTE, report.compute);
    if (report.estimation.state == HealthState::CRITICAL)
      choose(HealthAction::INHIBIT_ARM, HealthComponent::ESTIMATION, report.estimation);
    if (report.actuator.state == HealthState::CRITICAL)
      choose(HealthAction::INHIBIT_ARM, HealthComponent::ACTUATOR, report.actuator);
    if (report.config.state == HealthState::CRITICAL)
      choose(HealthAction::INHIBIT_ARM, HealthComponent::CONFIG, report.config);
  }
  else
  {
    if (report.power.state == HealthState::CRITICAL)
      choose(HealthAction::FORCE_LAND, HealthComponent::POWER, report.power);
    if (report.sensor.state == HealthState::CRITICAL)
      choose(HealthAction::FORCE_LAND, HealthComponent::SENSOR, report.sensor);
    if (report.estimation.state == HealthState::CRITICAL)
      choose(HealthAction::FORCE_LAND, HealthComponent::ESTIMATION, report.estimation);
    if (report.compute.state == HealthState::CRITICAL)
      choose(HealthAction::FORCE_LAND, HealthComponent::COMPUTE, report.compute);
    if (report.config.state == HealthState::CRITICAL)
      choose(HealthAction::FORCE_LAND, HealthComponent::CONFIG, report.config);
    if (report.actuator.state == HealthState::CRITICAL)
      choose(HealthAction::TERMINATE, HealthComponent::ACTUATOR, report.actuator);
  }

  if (report.requested_action == action && report.action_source == static_cast<uint8_t>(source) &&
      report.action_reason_mask == reasons)
  {
    return false;
  }
  report.requested_action = action;
  report.action_source = static_cast<uint8_t>(source);
  report.action_reason_mask = reasons;
  report.action_since_ms = context.now_ms;
  return true;
}
