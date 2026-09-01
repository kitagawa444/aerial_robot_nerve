#include "flashmemory/config_flash_database.h"

#include <cstring>

namespace
{
uint32_t updateCrc32(uint32_t crc, const uint8_t *data, size_t size)
{
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc;
}

bool flag(uint8_t value) { return value <= 1U; }
}  // namespace

void ConfigFlashDatabase::load(const ConfigFlashImage &slot_a, const ConfigFlashImage &slot_b)
{
  select_(slot_a, slot_b, true);
}

void ConfigFlashDatabase::reload(const ConfigFlashImage &slot_a, const ConfigFlashImage &slot_b)
{
  select_(slot_a, slot_b, false);
}

void ConfigFlashDatabase::select_(const ConfigFlashImage &slot_a, const ConfigFlashImage &slot_b, bool update_applied)
{
  const bool a_valid = validateImage(slot_a);
  const bool b_valid = validateImage(slot_b);
  if (!a_valid && !b_valid)
  {
    if (update_applied)
      loadDefaults();
    else
    {
      active_ = ConfigFlashImage{};
      active_.payload = applied_;
      staged_ = applied_;
      active_slot_ = SLOT_NONE;
      valid_ = false;
      dirty_ = false;
    }
    return;
  }

  if (a_valid && (!b_valid || generationNewer(slot_a.generation, slot_b.generation)))
  {
    active_ = slot_a;
    active_slot_ = SLOT_A;
  }
  else
  {
    active_ = slot_b;
    active_slot_ = SLOT_B;
  }
  staged_ = active_.payload;
  if (update_applied) applied_ = active_.payload;
  valid_ = true;
  dirty_ = false;
}

void ConfigFlashDatabase::loadDefaults(const ConfigFlashPayload &defaults)
{
  active_ = ConfigFlashImage{};
  active_.payload = defaults;
  staged_ = defaults;
  applied_ = defaults;
  active_slot_ = SLOT_NONE;
  valid_ = false;
  dirty_ = false;
}

bool ConfigFlashDatabase::stageConfiguration(const ConfigFlashPayload &payload, const ConfigFlashSupport &support)
{
  if (!validateConfiguration(payload, support)) return false;
  staged_ = payload;
  refreshDirty_();
  return true;
}

bool ConfigFlashDatabase::stageUart3Driver(uint8_t driver, bool gps_capable, bool crsf_capable)
{
  if (!validateDriver(driver, gps_capable, crsf_capable)) return false;
  staged_.uart3_driver = driver;
  refreshDirty_();
  return true;
}

bool ConfigFlashDatabase::prepareCommit(ConfigFlashImage &image, int8_t &target_slot) const
{
  if (!validateDependencies(staged_)) return false;
  image = ConfigFlashImage{};
  image.magic = MAGIC;
  image.schema_version = SCHEMA_VERSION;
  image.image_size = static_cast<uint16_t>(sizeof(ConfigFlashImage));
  image.generation = valid_ ? active_.generation + 1U : 1U;
  image.payload = staged_;
  image.crc32 = calculateCrc32(image);
  target_slot = active_slot_ == SLOT_A ? SLOT_B : SLOT_A;
  return validateImage(image);
}

bool ConfigFlashDatabase::acceptCommit(const ConfigFlashImage &image, int8_t slot)
{
  if ((slot != SLOT_A && slot != SLOT_B) || !validateImage(image)) return false;
  active_ = image;
  staged_ = image.payload;
  active_slot_ = slot;
  valid_ = true;
  dirty_ = false;
  return true;
}

bool ConfigFlashDatabase::rebootRequired() const { return std::memcmp(&staged_, &applied_, sizeof(staged_)) != 0; }

void ConfigFlashDatabase::setAppliedUart3Driver(uint8_t driver)
{
  if (driver <= Uart3Driver::CRSF) applied_.uart3_driver = driver;
}

uint32_t ConfigFlashDatabase::calculateCrc32(const ConfigFlashImage &image)
{
  uint32_t crc = 0xFFFFFFFFU;
  const auto *bytes = reinterpret_cast<const uint8_t *>(&image);
  crc = updateCrc32(crc, bytes, offsetof(ConfigFlashImage, crc32));
  return crc ^ 0xFFFFFFFFU;
}

bool ConfigFlashDatabase::validateImage(const ConfigFlashImage &image)
{
  return image.magic == MAGIC && image.schema_version == SCHEMA_VERSION &&
         image.image_size == sizeof(ConfigFlashImage) && validateDependencies(image.payload) &&
         image.crc32 == calculateCrc32(image);
}

bool ConfigFlashDatabase::validateConfiguration(const ConfigFlashPayload &payload, const ConfigFlashSupport &support)
{
  if (!validateDependencies(payload)) return false;
  if (payload.imu_driver == ImuDriver::MPU9250 && !support.imu_mpu9250) return false;
  if (payload.imu_driver == ImuDriver::ICM20948 && !support.imu_icm20948) return false;
  if (payload.barometer_enabled != 0U && !support.barometer) return false;
  if (payload.uart3_driver == Uart3Driver::GPS && !support.uart3_gps) return false;
  if (payload.uart3_driver == Uart3Driver::CRSF && !support.uart3_crsf) return false;
  if (payload.servo_driver == ServoDriver::DRIVER_DYNAMIXEL && !support.servo_dynamixel) return false;
  if (payload.servo_driver == ServoDriver::DRIVER_KONDO && !support.servo_kondo) return false;
  if (payload.motor_output_driver == MotorOutputDriver::DRIVER_PWM && !support.motor_pwm) return false;
  if (payload.motor_output_driver == MotorOutputDriver::DRIVER_DSHOT && !support.motor_dshot) return false;
  if (payload.attitude_estimation_enabled != 0U && !support.attitude_estimation) return false;
  if (payload.height_estimation_enabled != 0U && !support.height_estimation) return false;
  if (payload.position_estimation_enabled != 0U && !support.position_estimation) return false;
  if (payload.flight_control_enabled != 0U && !support.flight_control) return false;
  return true;
}

bool ConfigFlashDatabase::validateDependencies(const ConfigFlashPayload &payload)
{
  if (payload.imu_driver > ImuDriver::ICM20948 || payload.uart3_driver > Uart3Driver::CRSF ||
      payload.servo_driver > ServoDriver::DRIVER_KONDO ||
      payload.motor_output_driver > MotorOutputDriver::DRIVER_DSHOT || !flag(payload.barometer_enabled) ||
      !flag(payload.attitude_estimation_enabled) || !flag(payload.height_estimation_enabled) ||
      !flag(payload.position_estimation_enabled) || !flag(payload.flight_control_enabled))
  {
    return false;
  }

  const bool imu_enabled = payload.imu_driver != ImuDriver::DISABLED;
  if (payload.attitude_estimation_enabled != 0U && !imu_enabled) return false;
  if (payload.height_estimation_enabled != 0U &&
      (!imu_enabled || payload.barometer_enabled == 0U || payload.attitude_estimation_enabled == 0U))
  {
    return false;
  }
  if (payload.position_estimation_enabled != 0U && (!imu_enabled || payload.attitude_estimation_enabled == 0U))
  {
    return false;
  }
  if (payload.flight_control_enabled != 0U &&
      (payload.attitude_estimation_enabled == 0U || payload.motor_output_driver == MotorOutputDriver::DISABLED))
  {
    return false;
  }
  return true;
}

bool ConfigFlashDatabase::validateDriver(uint8_t driver, bool gps_capable, bool crsf_capable)
{
  if (driver == Uart3Driver::DISABLED) return true;
  if (driver == Uart3Driver::GPS) return gps_capable;
  if (driver == Uart3Driver::CRSF) return crsf_capable;
  return false;
}

bool ConfigFlashDatabase::generationNewer(uint32_t lhs, uint32_t rhs) { return static_cast<int32_t>(lhs - rhs) > 0; }

void ConfigFlashDatabase::refreshDirty_()
{
  dirty_ = !valid_ || std::memcmp(&staged_, &active_.payload, sizeof(staged_)) != 0;
}
