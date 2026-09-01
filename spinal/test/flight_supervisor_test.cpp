#include <gtest/gtest.h>

#include "flight_control/flight_control_types.h"
#include "flight_control/supervisor/flight_supervisor.h"

namespace
{
FlightSupervisorInput readyInput(uint32_t now_ms = 1U)
{
  FlightSupervisorInput input;
  input.now_ms = now_ms;
  input.attitude_ready = true;
  input.position_control_enabled = true;
  input.position_ready = true;
  input.estimation_ready = true;
  input.estimation_validity = 0x0FU;
  input.actuator_ready = true;
  input.config_ready = true;
  input.ros_link_state = FlightLinkState::CONNECTED;
  return input;
}

HealthManagerConfig configuredHealth()
{
  HealthManagerConfig config;
  config.power.battery_cell_count = 3U;
  config.power.low_percentage = 10.0f;
  config.power.hysteresis_percentage = 2.0f;
  config.power.high_cell_threshold = 0.5f;
  config.power.debounce_ms = 100U;
  return config;
}
}  // namespace

TEST(FlightSupervisor, RejectsArmUntilRequiredControllersAreReady)
{
  FlightSupervisor supervisor;
  FlightSupervisorInput input;
  EXPECT_FALSE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  EXPECT_EQ(supervisor.status().last_command_result, FlightCommandResult::ATTITUDE_NOT_READY);

  input.attitude_ready = true;
  input.position_control_enabled = true;
  EXPECT_FALSE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  EXPECT_EQ(supervisor.status().last_command_result, FlightCommandResult::POSITION_NOT_READY);

  input.position_ready = true;
  input.estimation_ready = true;
  input.actuator_ready = true;
  input.config_ready = true;
  EXPECT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  EXPECT_EQ(supervisor.status().arming_state, FlightArmingState::ARMED);
  EXPECT_EQ(supervisor.status().flight_phase, FlightPhase::LANDED);
}

TEST(FlightSupervisor, ReportsIndependentCommunicationLinks)
{
  FlightSupervisor supervisor;
  FlightSupervisorInput input = readyInput();
  input.ros_link_state = FlightLinkState::CONNECTED;
  input.network_link_state = FlightLinkState::STALE;
  input.rc_link_state = FlightLinkState::DISCONNECTED;

  supervisor.update(input);

  EXPECT_EQ(supervisor.status().ros_link_state, FlightLinkState::CONNECTED);
  EXPECT_EQ(supervisor.status().network_link_state, FlightLinkState::STALE);
  EXPECT_EQ(supervisor.status().rc_link_state, FlightLinkState::DISCONNECTED);
  EXPECT_EQ(supervisor.status().health.link.summary.state, HealthState::OK);
  EXPECT_EQ(supervisor.status().health.link.ros, FlightLinkState::CONNECTED);
  EXPECT_EQ(supervisor.status().health.link.network, FlightLinkState::STALE);
  EXPECT_EQ(supervisor.status().health.link.rc, FlightLinkState::DISCONNECTED);
}

TEST(FlightSupervisor, AggregatesComponentHealthWithoutMaskingUnknownMonitors)
{
  FlightSupervisor supervisor;
  FlightSupervisorInput input = readyInput(10U);
  supervisor.update(input);

  const FlightHealthReport &initial = supervisor.status().health;
  EXPECT_EQ(initial.flight.state, HealthState::OK);
  EXPECT_EQ(initial.estimation.state, HealthState::OK);
  EXPECT_EQ(initial.actuator.state, HealthState::OK);
  EXPECT_EQ(initial.config.state, HealthState::OK);
  EXPECT_EQ(initial.power.state, HealthState::UNKNOWN);
  EXPECT_EQ(initial.sensor.state, HealthState::UNKNOWN);
  EXPECT_EQ(initial.compute.state, HealthState::UNKNOWN);
  EXPECT_EQ(initial.overall.state, HealthState::OK);

  input.now_ms = 20U;
  input.ros_link_state = FlightLinkState::DISCONNECTED;
  input.network_link_state = FlightLinkState::DISCONNECTED;
  input.rc_link_state = FlightLinkState::DISCONNECTED;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().health.link.summary.state, HealthState::CRITICAL);
  EXPECT_EQ(supervisor.status().health.overall.state, HealthState::DEGRADED);

  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  input.now_ms = 30U;
  input.estimation_ready = false;
  input.position_ready = false;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().health.estimation.state, HealthState::CRITICAL);
  EXPECT_EQ(supervisor.status().health.overall.state, HealthState::CRITICAL);
}

TEST(FlightSupervisor, OwnsTakeoffCompletionTransition)
{
  FlightSupervisor supervisor;
  FlightSupervisorConfig config;
  config.takeoff_height = 1.0f;
  config.takeoff_stable_time_ms = 100U;
  ASSERT_TRUE(supervisor.configure(config));

  auto input = readyInput();
  input.vertical_position = 0.2f;
  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::RC, input));
  ASSERT_TRUE(supervisor.request(FlightControlCommand::TAKEOFF_CMD, FlightCommandSource::RC, input));
  EXPECT_EQ(supervisor.status().control_mode, FlightControlMode::TAKEOFF);
  EXPECT_FLOAT_EQ(supervisor.status().takeoff_target_height, 1.2f);

  input.now_ms = 10U;
  input.vertical_position = 1.2f;
  input.vertical_velocity = 0.0f;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().flight_phase, FlightPhase::TAKING_OFF);
  input.now_ms = 110U;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().flight_phase, FlightPhase::AIRBORNE);
  EXPECT_EQ(supervisor.status().control_mode, FlightControlMode::POSITION);
}

TEST(FlightSupervisor, NormalDisarmIsRejectedInAirButHaltIsAccepted)
{
  FlightSupervisor supervisor;
  auto input = readyInput();
  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  ASSERT_TRUE(supervisor.request(FlightControlCommand::TAKEOFF_CMD, FlightCommandSource::ROS, input));

  EXPECT_FALSE(supervisor.request(FlightControlCommand::ARM_OFF_CMD, FlightCommandSource::ROS, input));
  EXPECT_EQ(supervisor.status().last_command_result, FlightCommandResult::IN_AIR);
  EXPECT_EQ(supervisor.status().arming_state, FlightArmingState::ARMED);

  EXPECT_TRUE(supervisor.request(FlightControlCommand::HALT_CMD, FlightCommandSource::RC, input));
  EXPECT_EQ(supervisor.status().arming_state, FlightArmingState::DISARMED);
  EXPECT_EQ(supervisor.status().failsafe, FlightFailsafeState::TERMINATED);
  EXPECT_EQ(supervisor.status().health.flight.state, HealthState::CRITICAL);
  EXPECT_NE(supervisor.status().health.flight.reason_mask & HealthReason::TERMINATED, 0U);
}

TEST(FlightSupervisor, PositionLossSelectsForceLand)
{
  FlightSupervisor supervisor;
  auto input = readyInput();
  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  ASSERT_TRUE(supervisor.request(FlightControlCommand::TAKEOFF_CMD, FlightCommandSource::ROS, input));

  input.now_ms = 2U;
  input.position_ready = false;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().flight_phase, FlightPhase::LANDING);
  EXPECT_EQ(supervisor.status().control_mode, FlightControlMode::FORCE_LAND);
  EXPECT_EQ(supervisor.status().failsafe, FlightFailsafeState::FORCE_LAND);
  EXPECT_EQ(supervisor.status().transition_reason, FlightTransitionReason::POSITION_LOST);
  EXPECT_EQ(supervisor.status().health.flight.state, HealthState::CRITICAL);
}

TEST(FlightSupervisor, RcInputTemporarilyOwnsAuthority)
{
  FlightSupervisor supervisor;
  FlightSupervisorConfig config;
  config.takeoff_stable_time_ms = 1U;
  config.rc_authority_timeout_ms = 100U;
  ASSERT_TRUE(supervisor.configure(config));

  auto input = readyInput();
  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  ASSERT_TRUE(supervisor.request(FlightControlCommand::TAKEOFF_CMD, FlightCommandSource::ROS, input));
  input.vertical_position = supervisor.status().takeoff_target_height;
  input.now_ms = 10U;
  supervisor.update(input);
  input.now_ms = 11U;
  supervisor.update(input);
  ASSERT_EQ(supervisor.status().flight_phase, FlightPhase::AIRBORNE);

  supervisor.noteExternalSetpoint();
  EXPECT_EQ(supervisor.status().authority, FlightCommandSource::ROS);

  input.rc_connected = true;
  input.rc_active = true;
  input.now_ms = 20U;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().authority, FlightCommandSource::RC);

  supervisor.noteExternalSetpoint();
  EXPECT_EQ(supervisor.status().authority, FlightCommandSource::RC);

  input.rc_active = false;
  input.now_ms = 121U;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().authority, FlightCommandSource::INTERNAL);

  supervisor.noteExternalSetpoint();
  EXPECT_EQ(supervisor.status().authority, FlightCommandSource::ROS);
}

TEST(FlightSupervisor, NormalLandingAutoDisarmsAfterStableGroundDetection)
{
  FlightSupervisor supervisor;
  FlightSupervisorConfig config;
  config.landed_stable_time_ms = 100U;
  ASSERT_TRUE(supervisor.configure(config));
  auto input = readyInput();
  input.vertical_position = 0.3f;
  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  ASSERT_TRUE(supervisor.request(FlightControlCommand::TAKEOFF_CMD, FlightCommandSource::ROS, input));
  input.vertical_position = 1.3f;
  input.now_ms = 10U;
  supervisor.update(input);
  input.now_ms = 1010U;
  supervisor.update(input);
  ASSERT_EQ(supervisor.status().flight_phase, FlightPhase::AIRBORNE);
  ASSERT_TRUE(supervisor.request(FlightControlCommand::LAND_CMD, FlightCommandSource::ROS, input));
  EXPECT_FALSE(supervisor.request(FlightControlCommand::LAND_CMD, FlightCommandSource::ROS, input));
  EXPECT_EQ(supervisor.status().last_command_result, FlightCommandResult::ALREADY_IN_STATE);

  input.vertical_position = 0.35f;
  input.vertical_velocity = 0.0f;
  input.now_ms = 1020U;
  supervisor.update(input);
  input.now_ms = 1120U;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().arming_state, FlightArmingState::DISARMED);
  EXPECT_EQ(supervisor.status().user_intention, FlightControlMode::IDLE);
  EXPECT_EQ(supervisor.status().transition_reason, FlightTransitionReason::LANDING_COMPLETE);
}

TEST(FlightSupervisor, HealthPolicyInhibitsArmOnBatteryCellMismatch)
{
  FlightSupervisor supervisor;
  ASSERT_TRUE(supervisor.configureHealth(configuredHealth()));
  FlightSupervisorInput input = readyInput();
  input.power_monitor_present = true;
  input.battery_voltage = 16.8f;

  EXPECT_FALSE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  EXPECT_EQ(supervisor.status().last_command_result, FlightCommandResult::HEALTH_INHIBIT);
  EXPECT_EQ(supervisor.status().health.action_source, static_cast<uint8_t>(HealthComponent::POWER));
}

TEST(FlightSupervisor, HealthPolicyForceLandsAfterPersistentLowBattery)
{
  FlightSupervisor supervisor;
  ASSERT_TRUE(supervisor.configureHealth(configuredHealth()));
  FlightSupervisorInput input = readyInput(1U);
  input.power_monitor_present = true;
  input.battery_voltage = 12.0f;
  ASSERT_TRUE(supervisor.request(FlightControlCommand::ARM_ON_CMD, FlightCommandSource::ROS, input));
  ASSERT_TRUE(supervisor.request(FlightControlCommand::TAKEOFF_CMD, FlightCommandSource::ROS, input));

  input.battery_voltage = 10.8f;
  input.now_ms = 2U;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().failsafe, FlightFailsafeState::NONE);
  input.now_ms = 102U;
  supervisor.update(input);
  EXPECT_EQ(supervisor.status().failsafe, FlightFailsafeState::FORCE_LAND);
  EXPECT_EQ(supervisor.status().transition_reason, FlightTransitionReason::HEALTH_FORCE_LAND);
  EXPECT_EQ(supervisor.status().health.action_source, static_cast<uint8_t>(HealthComponent::POWER));
}
