#include <gtest/gtest.h>

#include "flight_control/health/health_manager.h"

namespace
{
HealthContext readyContext(uint32_t now_ms = 10U)
{
  HealthContext context;
  context.now_ms = now_ms;
  context.ready_to_arm = true;
  context.estimation_ready = true;
  context.estimation_validity = 0x0FU;
  context.actuator_ready = true;
  context.config_ready = true;
  context.ros_link_state = HealthLinkState::CONNECTED;
  return context;
}

HealthManagerConfig configuredHealth()
{
  HealthManagerConfig config;
  config.power.battery_cell_count = 3U;
  config.power.low_percentage = 10.0f;
  config.power.hysteresis_percentage = 2.0f;
  config.power.high_cell_threshold = 1.0f;
  config.power.debounce_ms = 100U;
  config.sensor.primary_imu_timeout_ms = 100U;
  config.compute.control_loop_deadline_ms = 10U;
  config.compute.miss_limit = 3U;
  return config;
}
}  // namespace

TEST(HealthManager, RunsStaticallyRegisteredMonitors)
{
  HealthManager manager;
  FlightHealthReport report;
  const HealthContext context = readyContext();

  EXPECT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.flight.state, HealthState::OK);
  EXPECT_EQ(report.link.summary.state, HealthState::OK);
  EXPECT_EQ(report.estimation.state, HealthState::OK);
  EXPECT_EQ(report.actuator.state, HealthState::OK);
  EXPECT_EQ(report.config.state, HealthState::OK);
  EXPECT_EQ(report.power.state, HealthState::UNKNOWN);
  EXPECT_EQ(report.sensor.state, HealthState::UNKNOWN);
  EXPECT_EQ(report.compute.state, HealthState::UNKNOWN);
  EXPECT_EQ(report.overall.state, HealthState::OK);
}

TEST(HealthManager, PreservesTimestampUntilEvaluationChanges)
{
  HealthManager manager;
  FlightHealthReport report;
  HealthContext context = readyContext(10U);
  ASSERT_TRUE(manager.update(context, report));
  const uint32_t initial_change_ms = report.estimation.last_change_ms;

  context.now_ms = 20U;
  EXPECT_FALSE(manager.update(context, report));
  EXPECT_EQ(report.estimation.last_change_ms, initial_change_ms);

  context.armed = true;
  context.estimation_ready = false;
  context.now_ms = 30U;
  EXPECT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.estimation.state, HealthState::CRITICAL);
  EXPECT_EQ(report.estimation.last_change_ms, 30U);
  EXPECT_EQ(report.overall.state, HealthState::CRITICAL);
}

TEST(HealthManager, ReportsLinkFailureAsDegradedOverallHealth)
{
  HealthManager manager;
  FlightHealthReport report;
  HealthContext context = readyContext();
  context.ros_link_state = HealthLinkState::DISCONNECTED;
  context.network_link_state = HealthLinkState::DISCONNECTED;
  context.rc_link_state = HealthLinkState::DISCONNECTED;

  ASSERT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.link.summary.state, HealthState::CRITICAL);
  EXPECT_EQ(report.link.summary.reason_mask, HealthReason::DISCONNECTED);
  EXPECT_EQ(report.overall.state, HealthState::DEGRADED);
  EXPECT_NE(report.overall.reason_mask & HealthReason::DISCONNECTED, 0U);
}

TEST(HealthManager, DebouncesBatteryThresholdAndRequestsForceLand)
{
  HealthManager manager;
  ASSERT_TRUE(manager.configure(configuredHealth()));
  FlightHealthReport report;
  HealthContext context = readyContext(10U);
  context.power_monitor_present = true;
  context.battery_voltage = 10.8f;
  context.armed = true;
  context.in_flight = true;

  ASSERT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.power.state, HealthState::DEGRADED);
  EXPECT_EQ(report.requested_action, HealthAction::NONE);
  EXPECT_GE(report.battery_percentage, 0.0f);

  context.now_ms = 110U;
  ASSERT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.power.state, HealthState::CRITICAL);
  EXPECT_EQ(report.requested_action, HealthAction::FORCE_LAND);
  EXPECT_EQ(report.action_source, static_cast<uint8_t>(HealthComponent::POWER));
}

TEST(HealthManager, MonitorsPrimaryImuFreshness)
{
  HealthManager manager;
  ASSERT_TRUE(manager.configure(configuredHealth()));
  FlightHealthReport report;
  HealthContext context = readyContext(200U);
  context.primary_imu_present = true;
  context.last_primary_imu_update_ms = 50U;

  ASSERT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.sensor.state, HealthState::CRITICAL);
  EXPECT_NE(report.sensor.reason_mask & HealthReason::STALE, 0U);
  EXPECT_EQ(report.requested_action, HealthAction::INHIBIT_ARM);
}

TEST(HealthManager, DebouncesControlLoopDeadlineMisses)
{
  HealthManager manager;
  ASSERT_TRUE(manager.configure(configuredHealth()));
  FlightHealthReport report;
  HealthContext context = readyContext(10U);
  context.armed = true;
  context.control_loop_period_valid = true;
  context.control_loop_period_ms = 20U;

  for (uint32_t sequence = 1U; sequence <= 2U; ++sequence)
  {
    context.control_loop_sequence = sequence;
    context.now_ms += 20U;
    (void)manager.update(context, report);
    EXPECT_EQ(report.compute.state, HealthState::DEGRADED);
  }
  context.control_loop_sequence = 3U;
  context.now_ms += 20U;
  ASSERT_TRUE(manager.update(context, report));
  EXPECT_EQ(report.compute.state, HealthState::CRITICAL);
  EXPECT_EQ(report.requested_action, HealthAction::FORCE_LAND);
}
