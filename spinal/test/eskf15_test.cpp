#include <gtest/gtest.h>

#include <cmath>

#include "state_estimate/eskf/eskf15.h"

namespace
{
constexpr float GRAVITY_MPS2 = 9.80665f;
}

TEST(Eskf15, StationaryPredictionRemainsStationary)
{
  Eskf15 filter;
  for (int index = 0; index < 1000; ++index)
  {
    ASSERT_TRUE(filter.predict(ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2), ap::Vector3f(), 0.001f));
  }

  EXPECT_NEAR(filter.state().position.x, 0.0f, 1.0e-4f);
  EXPECT_NEAR(filter.state().position.y, 0.0f, 1.0e-4f);
  EXPECT_NEAR(filter.state().position.z, 0.0f, 1.0e-4f);
  EXPECT_NEAR(filter.state().velocity.length(), 0.0f, 1.0e-4f);
}

TEST(Eskf15, PositionAndVelocityMeasurementsCorrectState)
{
  Eskf15 filter;
  ASSERT_TRUE(filter.predict(ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2), ap::Vector3f(), 0.01f));

  ASSERT_TRUE(filter.fusePosition(ap::Vector3f(1.0f, -2.0f, 0.5f), ap::Vector3f(0.01f, 0.01f, 0.01f)));
  ASSERT_TRUE(filter.fuseVelocity(ap::Vector3f(0.2f, -0.1f, 0.05f), ap::Vector3f(0.01f, 0.01f, 0.01f)));

  EXPECT_NEAR(filter.state().position.x, 1.0f, 0.02f);
  EXPECT_NEAR(filter.state().position.y, -2.0f, 0.02f);
  EXPECT_NEAR(filter.state().position.z, 0.5f, 0.02f);
  EXPECT_NEAR(filter.state().velocity.x, 0.2f, 0.02f);
  EXPECT_TRUE(filter.valid(Eskf15::HORIZONTAL_POSITION_VALID));
  EXPECT_TRUE(filter.valid(Eskf15::VERTICAL_POSITION_VALID));
  EXPECT_TRUE(filter.valid(Eskf15::VELOCITY_VALID));
}

TEST(Eskf15, AttitudeMeasurementCorrectsYaw)
{
  Eskf15 filter;
  ASSERT_TRUE(filter.predict(ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2), ap::Vector3f(), 0.01f));

  ap::Quaternion measured;
  measured.from_euler(0.0f, 0.0f, 0.2f);
  for (int index = 0; index < 5; ++index)
  {
    ASSERT_TRUE(filter.fuseAttitude(measured, ap::Vector3f(0.001f, 0.001f, 0.001f)));
  }

  EXPECT_NEAR(filter.state().attitude.get_euler_yaw(), 0.2f, 0.01f);
  EXPECT_TRUE(filter.valid(Eskf15::ATTITUDE_VALID));
}

TEST(Eskf15, InnovationGateRejectsOutlier)
{
  Eskf15 filter;
  ASSERT_TRUE(filter.predict(ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2), ap::Vector3f(), 0.01f));
  ASSERT_TRUE(filter.fuseAltitude(0.0f, 0.01f));

  EXPECT_FALSE(filter.fuseAltitude(1000.0f, 0.01f));
  EXPECT_EQ(filter.rejectedMeasurementCount(), 1U);
  EXPECT_LT(std::fabs(filter.state().position.z), 1.0f);
}

TEST(Eskf15, ResetPositionAndVelocityReacquiresExternalState)
{
  Eskf15 filter;
  ASSERT_TRUE(filter.predict(ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2), ap::Vector3f(), 0.01f));
  ASSERT_TRUE(filter.resetPosition(ap::Vector3f(4.0f, -3.0f, 2.0f), ap::Vector3f(0.04f, 0.04f, 0.09f)));
  ASSERT_TRUE(filter.resetVelocity(ap::Vector3f(0.3f, -0.2f, 0.1f), ap::Vector3f(0.01f, 0.01f, 0.01f)));

  EXPECT_FLOAT_EQ(filter.state().position.x, 4.0f);
  EXPECT_FLOAT_EQ(filter.state().position.y, -3.0f);
  EXPECT_FLOAT_EQ(filter.state().position.z, 2.0f);
  EXPECT_FLOAT_EQ(filter.state().velocity.x, 0.3f);
  EXPECT_FLOAT_EQ(filter.covariance(0, 0), 0.04f);
  EXPECT_FLOAT_EQ(filter.covariance(2, 2), 0.09f);
  EXPECT_FLOAT_EQ(filter.covariance(3, 3), 0.01f);
  EXPECT_TRUE(filter.valid(Eskf15::HORIZONTAL_POSITION_VALID));
  EXPECT_TRUE(filter.valid(Eskf15::VERTICAL_POSITION_VALID));
  EXPECT_TRUE(filter.valid(Eskf15::VELOCITY_VALID));
}
