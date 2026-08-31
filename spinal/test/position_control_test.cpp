#include <gtest/gtest.h>

#include "flight_control/position/position_control.h"

namespace
{
constexpr float GRAVITY_MPS2 = 9.80665f;

void makeEstimatorValid(Eskf15 &estimator)
{
  ASSERT_TRUE(estimator.predict(ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2), ap::Vector3f(), 0.01f));
  ASSERT_TRUE(estimator.fuseAttitude(ap::Quaternion(), ap::Vector3f(0.001f, 0.001f, 0.001f)));
  ASSERT_TRUE(estimator.fusePosition(ap::Vector3f(), ap::Vector3f(0.01f, 0.01f, 0.01f)));
  ASSERT_TRUE(estimator.fuseVelocity(ap::Vector3f(), ap::Vector3f(0.01f, 0.01f, 0.01f)));
}

PositionControlConfig config()
{
  PositionControlConfig result;
  result.motor_count = 4U;
  for (size_t index = 0; index < result.motor_count; ++index)
  {
    result.vertical_acceleration_to_thrust[index] = 0.05f;
  }
  return result;
}

PositionControlConfig lqiConfig()
{
  PositionControlConfig result = config();
  result.use_lqi_gains = true;
  result.yaw_rate_feedback_on_spinal = true;
  for (size_t index = 0; index < result.motor_count; ++index)
  {
    result.vertical_acceleration_to_thrust[index] = 0.25f;
    result.z_p_gain[index] = 2.0f;
    result.z_i_gain[index] = 3.0f;
    result.z_d_gain[index] = 4.0f;
    const float direction = index % 2U == 0U ? 1.0f : -1.0f;
    result.yaw_acceleration_to_thrust[index] = 3.0f * direction;
    result.yaw_p_gain[index] = 5.0f * direction;
    result.yaw_i_gain[index] = 1.0f * direction;
    result.yaw_d_gain[index] = 2.0f * direction;
  }
  return result;
}
}  // namespace

TEST(PositionController, HoverSetpointProducesLevelHoverThrust)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  ASSERT_TRUE(controller.configure(config()));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);
  EXPECT_TRUE(controller.active());

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 101U, command));
  EXPECT_NEAR(command.angles[0], 0.0f, 1.0e-5f);
  EXPECT_NEAR(command.angles[1], 0.0f, 1.0e-5f);
  ASSERT_EQ(command.base_thrust_count, 4U);
  EXPECT_NEAR(command.base_thrust[0], 0.05f * GRAVITY_MPS2, 1.0e-4f);
}

TEST(PositionController, ForwardPositionErrorCommandsPositivePitch)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  ASSERT_TRUE(controller.configure(config()));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.position.x = 1.0f;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 101U, command));
  EXPECT_GT(command.angles[1], 0.0f);
}

TEST(PositionController, LqiModeMatchesOriginalMotorSpaceZAndYawTerms)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  ASSERT_TRUE(controller.configure(lqiConfig()));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.position.z = 1.0f;
  setpoint.velocity.z = 0.5f;
  setpoint.acceleration.z = 0.25f;
  setpoint.yaw = 0.1f;
  setpoint.yaw_rate = 0.2f;
  setpoint.yaw_acceleration = 0.3f;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 101U, command));
  ASSERT_EQ(command.base_thrust_count, 4U);
  EXPECT_NEAR(command.base_thrust[0], 2.0f + 4.0f * 0.5f + 0.25f * 0.25f, 1.0e-5f);
  EXPECT_NEAR(command.angles[FlightControlAxis::Z], 5.0f * 0.1f + 2.0f * 0.2f + 3.0f * 0.3f, 1.0e-5f);
}

TEST(PositionController, LqiHorizontalLoopUsesOriginalPidLimitsAndControlMode)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionControlConfig controller_config = lqiConfig();
  controller_config.position_p[0] = 5.0f;
  controller_config.position_i[0] = 0.0f;
  controller_config.velocity_d[0] = 0.0f;
  controller_config.limit_err_p[0] = 0.2f;
  controller_config.limit_p[0] = 0.7f;

  PositionController controller;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.position.x = 10.0f;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 101U, command));
  EXPECT_NEAR(command.angles[FlightControlAxis::Y], 0.7f / GRAVITY_MPS2, 1.0e-5f);

  setpoint.horizontal_control_mode = 1U;
  controller.setSetpoint(setpoint, 102U);
  ASSERT_TRUE(controller.update(estimator, 102U, command));
  EXPECT_NEAR(command.angles[FlightControlAxis::Y], 0.0f, 1.0e-5f);
}

TEST(PositionController, LqiLandingUsesOriginalFixedDescentError)
{
  PositionController controller;
  PositionControlConfig controller_config = lqiConfig();
  controller_config.z_d_gain[0] = 0.0f;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);

  PositionControlSetpoint setpoint;
  setpoint.position.z = 0.0f;
  setpoint.landing = true;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  Eskf15::State state;
  state.attitude.initialise();
  state.position.z = 1.0f;
  const uint8_t validity = Eskf15::ATTITUDE_VALID | Eskf15::HORIZONTAL_POSITION_VALID |
                           Eskf15::VERTICAL_POSITION_VALID | Eskf15::VELOCITY_VALID;
  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(state, validity, 101U, command));
  EXPECT_NEAR(command.base_thrust[0], controller_config.z_p_gain[0] * controller_config.landing_err_z, 1.0e-5f);
  EXPECT_NEAR(controller.positionIntegral().z, 0.0f, 1.0e-5f);
}

TEST(PositionController, RequestsAttitudeIntegrationAfterOriginalHeightGate)
{
  PositionController controller;
  PositionControlConfig controller_config = lqiConfig();
  controller_config.start_roll_pitch_integration_height = 0.1f;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);

  PositionControlSetpoint setpoint;
  setpoint.initial_height = 0.2f;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  Eskf15::State state;
  state.position.z = 0.29f;
  EXPECT_FALSE(controller.shouldEnableAttitudeIntegration(state));
  state.position.z = 0.31f;
  EXPECT_TRUE(controller.shouldEnableAttitudeIntegration(state));
}

TEST(PositionController, AcceptsDirectStateWithValidityMask)
{
  PositionController controller;
  ASSERT_TRUE(controller.configure(config()));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  Eskf15::State direct_state;
  direct_state.attitude.initialise();
  const uint8_t validity = Eskf15::ATTITUDE_VALID | Eskf15::HORIZONTAL_POSITION_VALID |
                           Eskf15::VERTICAL_POSITION_VALID | Eskf15::VELOCITY_VALID;
  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(direct_state, validity, 101U, command));
  EXPECT_NEAR(command.base_thrust[0], 0.05f * GRAVITY_MPS2, 1.0e-4f);
}

TEST(PositionController, RejectsExpiredSetpoint)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  PositionControlConfig controller_config = config();
  controller_config.setpoint_timeout_ms = 100U;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  FlightControlFourAxisCommand command;
  EXPECT_FALSE(controller.update(estimator, 201U, command));
}

TEST(PositionController, RepeatedIdenticalConfigPreservesSetpoint)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  const PositionControlConfig controller_config = config();
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  ASSERT_TRUE(controller.configure(controller_config));
  EXPECT_TRUE(controller.ready(estimator, 101U));
  EXPECT_TRUE(controller.active());
}

TEST(PositionController, CompatibleGainUpdatePreservesSetpoint)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  PositionControlConfig controller_config = lqiConfig();
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  controller.setSetpoint(setpoint, 100U);

  controller_config.z_p_gain[0] += 0.1f;
  ASSERT_TRUE(controller.canReconfigureInFlight(controller_config));
  ASSERT_TRUE(controller.configure(controller_config));
  EXPECT_TRUE(controller.ready(estimator, 101U));
  EXPECT_TRUE(controller.active());
}

TEST(PositionController, ManualForwardInputMovesBodyFrameSetpoint)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  PositionControlConfig controller_config = config();
  controller_config.setpoint_timeout_ms = 1000U;
  controller_config.rc_timeout_ms = 1000U;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  setpoint.manual_control_allowed = true;
  controller.setSetpoint(setpoint, 100U);
  PositionControlRcInput rc_input;
  rc_input.forward = 1.0f;
  rc_input.connected = true;
  controller.setRcInput(rc_input, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 100U, command));
  ASSERT_TRUE(controller.update(estimator, 200U, command));

  EXPECT_NEAR(controller.rcPositionOffset().x, 0.05f, 1.0e-5f);
  EXPECT_NEAR(controller.rcPositionOffset().y, 0.0f, 1.0e-5f);
  EXPECT_GT(command.angles[FlightControlAxis::Y], 0.0f);
}

TEST(PositionController, ManualInputIsIgnoredOutsideHover)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  ASSERT_TRUE(controller.configure(config()));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  setpoint.manual_control_allowed = false;
  controller.setSetpoint(setpoint, 100U);
  PositionControlRcInput rc_input;
  rc_input.forward = 1.0f;
  rc_input.connected = true;
  controller.setRcInput(rc_input, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 100U, command));
  ASSERT_TRUE(controller.update(estimator, 200U, command));

  EXPECT_NEAR(controller.rcPositionOffset().x, 0.0f, 1.0e-5f);
  EXPECT_NEAR(command.angles[FlightControlAxis::Y], 0.0f, 1.0e-5f);
}

TEST(PositionController, RcTimeoutHoldsReachedSetpoint)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  PositionControlConfig controller_config = config();
  controller_config.setpoint_timeout_ms = 1000U;
  controller_config.rc_timeout_ms = 100U;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  setpoint.manual_control_allowed = true;
  controller.setSetpoint(setpoint, 100U);
  PositionControlRcInput rc_input;
  rc_input.forward = 1.0f;
  rc_input.connected = true;
  controller.setRcInput(rc_input, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 100U, command));
  ASSERT_TRUE(controller.update(estimator, 200U, command));
  const float offset_before_timeout = controller.rcPositionOffset().x;
  ASSERT_TRUE(controller.update(estimator, 301U, command));

  EXPECT_GT(offset_before_timeout, 0.0f);
  EXPECT_NEAR(controller.rcPositionOffset().x, offset_before_timeout, 1.0e-5f);
}

TEST(PositionController, LeavingHoverClearsOnlyAltitudeOffset)
{
  Eskf15 estimator;
  makeEstimatorValid(estimator);

  PositionController controller;
  PositionControlConfig controller_config = config();
  controller_config.setpoint_timeout_ms = 1000U;
  controller_config.rc_timeout_ms = 1000U;
  ASSERT_TRUE(controller.configure(controller_config));
  controller.setEnabled(true);
  PositionControlSetpoint setpoint;
  setpoint.active = true;
  setpoint.manual_control_allowed = true;
  controller.setSetpoint(setpoint, 100U);
  PositionControlRcInput rc_input;
  rc_input.forward = 1.0f;
  rc_input.vertical = 1.0f;
  rc_input.yaw = 1.0f;
  rc_input.connected = true;
  controller.setRcInput(rc_input, 100U);

  FlightControlFourAxisCommand command;
  ASSERT_TRUE(controller.update(estimator, 100U, command));
  ASSERT_TRUE(controller.update(estimator, 200U, command));
  const float horizontal_offset = controller.rcPositionOffset().x;
  const float yaw_offset = controller.rcYawOffset();
  ASSERT_GT(controller.rcPositionOffset().z, 0.0f);

  setpoint.manual_control_allowed = false;
  controller.setSetpoint(setpoint, 201U);

  EXPECT_NEAR(controller.rcPositionOffset().x, horizontal_offset, 1.0e-5f);
  EXPECT_NEAR(controller.rcPositionOffset().z, 0.0f, 1.0e-5f);
  EXPECT_NEAR(controller.rcYawOffset(), yaw_offset, 1.0e-5f);
}
