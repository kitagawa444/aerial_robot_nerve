#include <gtest/gtest.h>

#include "flashmemory/config_flash_database.h"
#include "flashmemory/config_flash_storage_sim.h"

#include <cstdio>

namespace
{
ConfigFlashImage image(uint32_t generation, uint8_t driver)
{
  ConfigFlashImage result;
  result.magic = ConfigFlashDatabase::MAGIC;
  result.schema_version = ConfigFlashDatabase::SCHEMA_VERSION;
  result.image_size = sizeof(ConfigFlashImage);
  result.generation = generation;
  result.payload.uart3_driver = driver;
  result.crc32 = ConfigFlashDatabase::calculateCrc32(result);
  return result;
}
}  // namespace

TEST(ConfigFlashDatabase, UsesDefaultsWhenBothSlotsInvalid)
{
  ConfigFlashDatabase database;
  database.load(ConfigFlashImage{}, ConfigFlashImage{});
  EXPECT_FALSE(database.valid());
  EXPECT_FALSE(database.dirty());
  EXPECT_EQ(database.activeSlot(), ConfigFlashDatabase::SLOT_NONE);
  EXPECT_EQ(database.activeUart3Driver(), Uart3Driver::CRSF);
}

TEST(ConfigFlashDatabase, SelectsNewestValidSlot)
{
  ConfigFlashDatabase database;
  database.load(image(4U, Uart3Driver::GPS), image(5U, Uart3Driver::CRSF));
  EXPECT_TRUE(database.valid());
  EXPECT_EQ(database.activeSlot(), ConfigFlashDatabase::SLOT_B);
  EXPECT_EQ(database.activeUart3Driver(), Uart3Driver::CRSF);
}

TEST(ConfigFlashDatabase, FallsBackWhenNewestSlotIsCorrupt)
{
  ConfigFlashImage corrupt = image(7U, Uart3Driver::CRSF);
  corrupt.crc32 ^= 1U;
  ConfigFlashDatabase database;
  database.load(image(6U, Uart3Driver::GPS), corrupt);
  EXPECT_TRUE(database.valid());
  EXPECT_EQ(database.activeSlot(), ConfigFlashDatabase::SLOT_A);
  EXPECT_EQ(database.activeUart3Driver(), Uart3Driver::GPS);
  EXPECT_EQ(database.pendingUart3Driver(), Uart3Driver::GPS);
}

TEST(ConfigFlashDatabase, CommitAlternatesSlotsAndClearsDirtyState)
{
  ConfigFlashDatabase database;
  database.load(image(2U, Uart3Driver::CRSF), ConfigFlashImage{});
  ASSERT_TRUE(database.stageUart3Driver(Uart3Driver::GPS, true, true));
  EXPECT_TRUE(database.dirty());
  EXPECT_TRUE(database.rebootRequired());

  ConfigFlashImage pending;
  int8_t slot = ConfigFlashDatabase::SLOT_NONE;
  ASSERT_TRUE(database.prepareCommit(pending, slot));
  EXPECT_EQ(slot, ConfigFlashDatabase::SLOT_B);
  EXPECT_EQ(pending.generation, 3U);
  ASSERT_TRUE(database.acceptCommit(pending, slot));
  EXPECT_FALSE(database.dirty());
  EXPECT_TRUE(database.rebootRequired());
  EXPECT_EQ(database.activeUart3Driver(), Uart3Driver::CRSF);
  EXPECT_EQ(database.pendingUart3Driver(), Uart3Driver::GPS);
}

TEST(ConfigFlashDatabase, RejectsDriverMissingFromApplication)
{
  ConfigFlashDatabase database;
  database.loadDefaults();
  EXPECT_FALSE(database.stageUart3Driver(Uart3Driver::GPS, false, true));
  EXPECT_TRUE(database.stageUart3Driver(Uart3Driver::CRSF, false, true));
}

TEST(ConfigFlashDatabase, RuntimeDriverDoesNotChangeUntilBoot)
{
  ConfigFlashDatabase database;
  database.load(image(1U, Uart3Driver::CRSF), ConfigFlashImage{});
  ASSERT_TRUE(database.stageUart3Driver(Uart3Driver::GPS, true, true));
  ConfigFlashImage committed;
  int8_t slot = ConfigFlashDatabase::SLOT_NONE;
  ASSERT_TRUE(database.prepareCommit(committed, slot));
  ASSERT_TRUE(database.acceptCommit(committed, slot));
  EXPECT_EQ(database.activeUart3Driver(), Uart3Driver::CRSF);
  EXPECT_EQ(database.pendingUart3Driver(), Uart3Driver::GPS);
  EXPECT_TRUE(database.rebootRequired());

  ConfigFlashDatabase after_reboot;
  after_reboot.load(image(1U, Uart3Driver::CRSF), committed);
  EXPECT_EQ(after_reboot.activeUart3Driver(), Uart3Driver::GPS);
  EXPECT_FALSE(after_reboot.rebootRequired());
}

TEST(ConfigFlashDatabase, StagesEveryBootConfigurationField)
{
  ConfigFlashDatabase database;
  database.loadDefaults();
  ConfigFlashPayload pending = database.pendingConfiguration();
  pending.imu_driver = ImuDriver::MPU9250;
  pending.barometer_enabled = 0U;
  pending.uart3_driver = Uart3Driver::GPS;
  pending.servo_driver = ServoDriver::DRIVER_KONDO;
  pending.height_estimation_enabled = 0U;
  pending.motor_output_driver = MotorOutputDriver::DRIVER_PWM;

  ConfigFlashSupport support;
  support.imu_mpu9250 = true;
  support.uart3_gps = true;
  support.servo_kondo = true;
  support.motor_pwm = true;
  support.attitude_estimation = true;
  support.position_estimation = true;
  support.flight_control = true;
  ASSERT_TRUE(database.stageConfiguration(pending, support));
  EXPECT_TRUE(database.dirty());
  EXPECT_TRUE(database.rebootRequired());
  EXPECT_EQ(database.pendingConfiguration().imu_driver, ImuDriver::MPU9250);
  EXPECT_EQ(database.pendingConfiguration().servo_driver, ServoDriver::DRIVER_KONDO);
  EXPECT_EQ(database.pendingConfiguration().motor_output_driver, MotorOutputDriver::DRIVER_PWM);
}

TEST(ConfigFlashDatabase, RejectsInvalidSubsystemDependencies)
{
  ConfigFlashDatabase database;
  database.loadDefaults();
  ConfigFlashPayload pending = database.pendingConfiguration();
  pending.imu_driver = ImuDriver::DISABLED;
  EXPECT_FALSE(ConfigFlashDatabase::validateDependencies(pending));

  pending = database.pendingConfiguration();
  pending.barometer_enabled = 0U;
  EXPECT_FALSE(ConfigFlashDatabase::validateDependencies(pending));

  pending = database.pendingConfiguration();
  pending.motor_output_driver = MotorOutputDriver::DISABLED;
  EXPECT_FALSE(ConfigFlashDatabase::validateDependencies(pending));
}

TEST(ConfigFlashStorageSim, PersistsBothSlotsAcrossInstances)
{
  const std::string path = "/tmp/spinal_config_flash_test.bin";
  (void)std::remove(path.c_str());

  ConfigFlashDatabase first;
  ConfigFlashStorageSim first_storage;
  EXPECT_FALSE(first_storage.init(path, first));
  first.loadDefaults();
  ConfigFlashPayload pending = first.pendingConfiguration();
  pending.uart3_driver = Uart3Driver::GPS;
  ConfigFlashSupport support;
  support.imu_icm20948 = true;
  support.barometer = true;
  support.uart3_gps = true;
  support.servo_dynamixel = true;
  support.motor_dshot = true;
  support.attitude_estimation = true;
  support.height_estimation = true;
  support.position_estimation = true;
  support.flight_control = true;
  ASSERT_TRUE(first.stageConfiguration(pending, support));
  ASSERT_TRUE(first_storage.commit(first));

  ConfigFlashDatabase second;
  ConfigFlashStorageSim second_storage;
  ASSERT_TRUE(second_storage.init(path, second));
  EXPECT_EQ(second.activeUart3Driver(), Uart3Driver::GPS);
  EXPECT_TRUE(second.valid());
  (void)std::remove(path.c_str());
}
