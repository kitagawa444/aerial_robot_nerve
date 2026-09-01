#include <gtest/gtest.h>

#include "flight_control/attitude/attitude_control.h"

namespace
{
AttitudeController configuredController()
{
  AttitudeController controller;
  controller.setMotorNumber(4U);
  controller.setUavModel(FlightControlUavModel::DRONE);
  ThrusterControlLimits limits;
  limits.configured = true;
  limits.max_thrust = 8.0f;
  controller.setThrusterLimits(limits);
  return controller;
}

FlightControlRpyTerms perMotorGains()
{
  FlightControlRpyTerms gains;
  gains.motors_count = 4U;
  for (size_t motor = 0; motor < gains.motors_count; ++motor)
  {
    gains.motors[motor].roll_p = 100;
    gains.motors[motor].pitch_p = 100;
    gains.motors[motor].yaw_d = 100;
  }
  return gains;
}
}

TEST(AttitudeController, RejectsActivationUntilRpyGainsAreConfigured)
{
  AttitudeController controller = configuredController();
  EXPECT_FALSE(controller.activated());

  ASSERT_TRUE(controller.applyRpyGains(perMotorGains()));
  EXPECT_TRUE(controller.activated());
}

TEST(AttitudeController, PreservesStaticGainsAcrossDisarmReset)
{
  AttitudeController controller = configuredController();
  ASSERT_TRUE(controller.applyRpyGains(perMotorGains()));
  controller.setStartControlFlag(true);
  controller.setStartControlFlag(false);

  EXPECT_TRUE(controller.rpyGainsConfigured());
  EXPECT_TRUE(controller.activated());
}

TEST(AttitudeController, GlobalGainsRequirePreArmAllocation)
{
  AttitudeController controller = configuredController();
  FlightControlRpyTerms global_gains;
  global_gains.motors_count = 1U;
  ASSERT_TRUE(controller.applyRpyGains(global_gains));
  EXPECT_FALSE(controller.activated());

  FlightControlTorqueAllocationMatrixInv allocation;
  allocation.rows_count = 4U;
  ASSERT_TRUE(controller.applyTorqueAllocationMatrixInv(allocation));
  EXPECT_TRUE(controller.activated());

  controller.setStartControlFlag(true);
  EXPECT_FALSE(controller.applyTorqueAllocationMatrixInv(allocation));
}
