#include <gtest/gtest.h>

#include "state_estimate/state_estimate.h"

namespace
{
constexpr uint8_t ALL_EXTERNAL_FIELDS = 1U | 2U | 4U;
}

TEST(StateEstimate, DirectExternalStateBypassesEskf)
{
  StateEstimate estimator;
  estimator.init();
  estimator.update(100U);

  ap::Quaternion attitude;
  attitude.from_euler(0.1f, -0.2f, 0.3f);
  ASSERT_TRUE(estimator.applyDirectExternalState(ap::Vector3f(1.0f, 2.0f, 3.0f), ap::Vector3f(0.01f, 0.02f, 0.03f),
                                                 ap::Vector3f(0.4f, 0.5f, 0.6f), ap::Vector3f(0.04f, 0.05f, 0.06f),
                                                 attitude, ap::Vector3f(0.001f, 0.002f, 0.003f),
                                                 ap::Vector3f(0.7f, 0.8f, 0.9f), ALL_EXTERNAL_FIELDS, 100U));

  EXPECT_TRUE(estimator.directStateEnabled());
  EXPECT_FLOAT_EQ(estimator.outputState().position.x, 1.0f);
  EXPECT_FLOAT_EQ(estimator.outputState().velocity.z, 0.6f);
  EXPECT_NEAR(estimator.outputState().attitude.get_euler_yaw(), 0.3f, 1.0e-5f);
  EXPECT_FLOAT_EQ(estimator.getAngularRate().y, 0.8f);
  EXPECT_FLOAT_EQ(estimator.outputCovariance(1U), 0.02f);
  EXPECT_EQ(estimator.stateValidity(), Eskf15::ATTITUDE_VALID | Eskf15::HORIZONTAL_POSITION_VALID |
                                           Eskf15::VERTICAL_POSITION_VALID | Eskf15::VELOCITY_VALID);

  // A direct measurement must not be injected into the ESKF state.
  EXPECT_NE(estimator.getEskf()->state().position.x, estimator.outputState().position.x);
}

TEST(StateEstimate, DirectExternalStateExpires)
{
  StateEstimate estimator;
  estimator.init();
  estimator.update(100U);

  ap::Quaternion attitude;
  attitude.initialise();
  ASSERT_TRUE(estimator.applyDirectExternalState(
      ap::Vector3f(), ap::Vector3f(0.01f, 0.01f, 0.01f), ap::Vector3f(), ap::Vector3f(0.01f, 0.01f, 0.01f), attitude,
      ap::Vector3f(0.01f, 0.01f, 0.01f), ap::Vector3f(), ALL_EXTERNAL_FIELDS, 100U));

  EXPECT_NE(estimator.stateValidity(), 0U);
  estimator.update(1201U);
  EXPECT_EQ(estimator.stateValidity(), 0U);
  EXPECT_FALSE(estimator.positionControlStateValid());
}

TEST(StateEstimate, DirectExternalTimestampMayLeadImuTick)
{
  StateEstimate estimator;
  estimator.init();
  estimator.update(100U);

  ap::Quaternion attitude;
  attitude.initialise();
  ASSERT_TRUE(estimator.applyDirectExternalState(
      ap::Vector3f(), ap::Vector3f(0.01f, 0.01f, 0.01f), ap::Vector3f(), ap::Vector3f(0.01f, 0.01f, 0.01f), attitude,
      ap::Vector3f(0.01f, 0.01f, 0.01f), ap::Vector3f(), ALL_EXTERNAL_FIELDS, 101U));

  // The ROS clock can advance between the IMU update and external-state
  // callback in the same controller cycle. A one-tick lead is fresh, not a
  // uint32 timeout caused by subtraction underflow.
  EXPECT_TRUE(estimator.positionControlStateValid());
  estimator.update(102U);
  EXPECT_TRUE(estimator.positionControlStateValid());
}
