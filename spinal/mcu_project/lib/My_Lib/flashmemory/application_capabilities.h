#pragma once

#include <cstdint>

#include "config.h"
#include "flashmemory/config_flash_database.h"

namespace ApplicationCapability
{
enum : uint32_t
{
  IMU_ICM20948 = 1U << 0,
  IMU_MPU9250 = 1U << 1,
  BAROMETER = 1U << 2,
  GPS_UART3 = 1U << 3,
  CRSF_UART3 = 1U << 4,
  DYNAMIXEL_DRIVER = 1U << 5,
  KONDO_DRIVER = 1U << 6,
  DSHOT_OUTPUT = 1U << 7,
  FLIGHT_CONTROL = 1U << 8,
  POSITION_CONTROL = 1U << 9,
  CONFIG_FLASH_AB = 1U << 10,
  PWM_OUTPUT = 1U << 11,
  ATTITUDE_ESTIMATION = 1U << 12,
  HEIGHT_ESTIMATION = 1U << 13,
  POSITION_ESTIMATION = 1U << 14,
};

constexpr uint32_t mask()
{
  return (CAPABILITY_IMU_ICM20948 ? IMU_ICM20948 : 0U) | (CAPABILITY_IMU_MPU9250 ? IMU_MPU9250 : 0U) |
         (CAPABILITY_BAROMETER ? BAROMETER : 0U) | (CAPABILITY_GPS_UART3 ? GPS_UART3 : 0U) |
         (CAPABILITY_CRSF_UART3 ? CRSF_UART3 : 0U) | (CAPABILITY_DYNAMIXEL ? DYNAMIXEL_DRIVER : 0U) |
         (CAPABILITY_KONDO ? KONDO_DRIVER : 0U) | (CAPABILITY_DSHOT ? DSHOT_OUTPUT : 0U) |
         (CAPABILITY_PWM ? PWM_OUTPUT : 0U) | (CAPABILITY_FLIGHT_CONTROL ? FLIGHT_CONTROL : 0U) | POSITION_CONTROL |
         CONFIG_FLASH_AB | (CAPABILITY_ATTITUDE_ESTIMATION ? ATTITUDE_ESTIMATION : 0U) |
         (CAPABILITY_HEIGHT_ESTIMATION ? HEIGHT_ESTIMATION : 0U) |
         (CAPABILITY_POSITION_ESTIMATION ? POSITION_ESTIMATION : 0U);
}

constexpr bool has(uint32_t capability) { return (mask() & capability) != 0U; }

inline ConfigFlashSupport support()
{
  ConfigFlashSupport result;
  result.imu_mpu9250 = CAPABILITY_IMU_MPU9250 != 0;
  result.imu_icm20948 = CAPABILITY_IMU_ICM20948 != 0;
  result.barometer = CAPABILITY_BAROMETER != 0;
  result.uart3_gps = CAPABILITY_GPS_UART3 != 0;
  result.uart3_crsf = CAPABILITY_CRSF_UART3 != 0;
  result.servo_dynamixel = CAPABILITY_DYNAMIXEL != 0;
  result.servo_kondo = CAPABILITY_KONDO != 0;
  result.motor_pwm = CAPABILITY_PWM != 0;
  result.motor_dshot = CAPABILITY_DSHOT != 0;
  result.attitude_estimation = CAPABILITY_ATTITUDE_ESTIMATION != 0;
  result.height_estimation = CAPABILITY_HEIGHT_ESTIMATION != 0;
  result.position_estimation = CAPABILITY_POSITION_ESTIMATION != 0;
  result.flight_control = CAPABILITY_FLIGHT_CONTROL != 0;
  return result;
}

inline ConfigFlashPayload defaults()
{
  ConfigFlashPayload payload{};
  payload.imu_driver = DEFAULT_IMU_DRIVER;
  payload.barometer_enabled = DEFAULT_BAROMETER_ENABLED;
  payload.uart3_driver = DEFAULT_UART3_DRIVER;
  payload.servo_driver = DEFAULT_SERVO_DRIVER;
  payload.attitude_estimation_enabled = DEFAULT_ATTITUDE_ESTIMATION_ENABLED;
  payload.height_estimation_enabled = DEFAULT_HEIGHT_ESTIMATION_ENABLED;
  payload.position_estimation_enabled = DEFAULT_POSITION_ESTIMATION_ENABLED;
  payload.flight_control_enabled = DEFAULT_FLIGHT_CONTROL_ENABLED;
  payload.motor_output_driver = DEFAULT_MOTOR_OUTPUT_DRIVER;
  return payload;
}
}  // namespace ApplicationCapability
