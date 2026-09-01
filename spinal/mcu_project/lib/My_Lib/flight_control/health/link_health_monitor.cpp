#include "flight_control/health/link_health_monitor.h"

HealthEvaluation LinkHealthMonitor::evaluate(const HealthContext &context)
{
  const uint8_t links[] = { context.ros_link_state, context.network_link_state, context.rc_link_state };
  bool connected = false;
  bool connecting = false;
  bool stale = false;
  bool unknown = false;
  bool disconnected = false;
  bool all_disabled = true;

  for (uint8_t link : links)
  {
    connected |= link == HealthLinkState::CONNECTED;
    connecting |= link == HealthLinkState::CONNECTING;
    stale |= link == HealthLinkState::STALE;
    unknown |= link == HealthLinkState::UNKNOWN;
    disconnected |= link == HealthLinkState::DISCONNECTED;
    all_disabled &= link == HealthLinkState::DISABLED;
  }

  if (connected) return { HealthState::OK, HealthReason::NONE };
  if (stale || connecting)
  {
    uint32_t reasons = HealthReason::NONE;
    if (stale) reasons |= HealthReason::STALE;
    if (connecting) reasons |= HealthReason::NOT_READY;
    return { HealthState::DEGRADED, reasons };
  }
  if (all_disabled) return { HealthState::DISABLED, HealthReason::NONE };
  if (unknown)
  {
    uint32_t reasons = HealthReason::DATA_UNAVAILABLE;
    if (disconnected) reasons |= HealthReason::DISCONNECTED;
    return { HealthState::UNKNOWN, reasons };
  }
  return { HealthState::CRITICAL, HealthReason::DISCONNECTED };
}
