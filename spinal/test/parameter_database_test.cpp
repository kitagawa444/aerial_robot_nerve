#include <gtest/gtest.h>

#include <cstring>

#include "flashmemory/parameter_database.h"

namespace
{
FlightParameterPayload validPayload()
{
  FlightParameterPayload payload;
  payload.valid_fields = FlightParameterField::AIRFRAME | FlightParameterField::PWM |
                         FlightParameterField::ATTITUDE_GAINS | FlightParameterField::TORQUE_ALLOCATION;
  payload.motor_count = 4U;
  payload.uav_model = FlightControlUavModel::DRONE;
  payload.pwm.min_pwm = 0.5f;
  payload.pwm.max_pwm = 0.9f;
  payload.pwm.force_landing_thrust = 2.0f;
  payload.pwm.motor_info_count = 1U;
  payload.pwm.motor_info[0].voltage = 12.0f;
  payload.pwm.motor_info[0].max_thrust = 10.0f;
  payload.attitude_gains.motors_count = 1U;
  payload.torque_allocation.rows_count = 4U;
  return payload;
}
}  // namespace

TEST(FlightParameterDatabase, RejectsErasedStorage)
{
  FlightParameterDatabase database;
  std::memset(database.storageData(), 0xFF, database.storageSize());
  database.load();
  EXPECT_FALSE(database.valid());
  EXPECT_EQ(database.generation(), 0U);
}

TEST(FlightParameterDatabase, CommitsAndReloadsValidatedImage)
{
  FlightParameterDatabase database;
  ASSERT_TRUE(database.stage(validPayload()));
  ASSERT_TRUE(database.prepareCommit());
  EXPECT_TRUE(database.valid());
  EXPECT_EQ(database.generation(), 1U);
  EXPECT_NE(database.crc32(), 0U);

  database.load();
  EXPECT_TRUE(database.valid());
  EXPECT_FALSE(database.dirty());
  EXPECT_EQ(database.payload().motor_count, 4U);
}

TEST(FlightParameterDatabase, DetectsPayloadCorruption)
{
  FlightParameterDatabase database;
  ASSERT_TRUE(database.stage(validPayload()));
  ASSERT_TRUE(database.prepareCommit());

  auto *bytes = static_cast<uint8_t *>(database.storageData());
  bytes[database.storageSize() - 1U] ^= 0x01U;
  database.load();
  EXPECT_FALSE(database.valid());
}

TEST(FlightParameterDatabase, IncrementsGeneration)
{
  FlightParameterDatabase database;
  ASSERT_TRUE(database.stage(validPayload()));
  ASSERT_TRUE(database.prepareCommit());
  ASSERT_TRUE(database.stage(validPayload()));
  ASSERT_TRUE(database.prepareCommit());
  EXPECT_EQ(database.generation(), 2U);
}

TEST(FlightParameterDatabase, RejectsIncompleteConfiguration)
{
  FlightParameterDatabase database;
  FlightParameterPayload payload = validPayload();
  payload.valid_fields &= ~FlightParameterField::PWM;
  EXPECT_TRUE(database.stage(payload));
  EXPECT_FALSE(database.prepareCommit());
}
