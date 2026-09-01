#include "flashmemory/parameter_database.h"

#include <cmath>
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

bool finite(float value) { return std::isfinite(value); }
}  // namespace

void FlightParameterDatabase::load()
{
  valid_ = validateImage(image_);
  if (valid_)
    staged_ = image_.payload;
  else
    staged_ = FlightParameterPayload{};
  dirty_ = false;
}

bool FlightParameterDatabase::stage(const FlightParameterPayload &payload)
{
  staged_ = payload;
  dirty_ = !valid_ || std::memcmp(&staged_, &image_.payload, sizeof(staged_)) != 0;
  return true;
}

bool FlightParameterDatabase::prepareCommit()
{
  if (!validatePayload(staged_)) return false;

  const uint32_t next_generation = valid_ ? image_.generation + 1U : 1U;
  image_ = FlightParameterImage{};
  image_.magic = MAGIC;
  image_.schema_version = SCHEMA_VERSION;
  image_.image_size = static_cast<uint16_t>(sizeof(FlightParameterImage));
  image_.generation = next_generation;
  image_.payload = staged_;
  image_.crc32 = calculateCrc32(image_);
  valid_ = true;
  dirty_ = false;
  return true;
}

uint32_t FlightParameterDatabase::calculateCrc32(const FlightParameterImage &image)
{
  uint32_t crc = 0xFFFFFFFFU;
  const auto *bytes = reinterpret_cast<const uint8_t *>(&image);
  crc = updateCrc32(crc, bytes, offsetof(FlightParameterImage, crc32));
  crc = updateCrc32(crc, bytes + offsetof(FlightParameterImage, payload), sizeof(image.payload));
  return crc ^ 0xFFFFFFFFU;
}

bool FlightParameterDatabase::validateImage(const FlightParameterImage &image)
{
  if (image.magic != MAGIC || image.schema_version != SCHEMA_VERSION ||
      image.image_size != sizeof(FlightParameterImage))
    return false;
  if (!validatePayload(image.payload)) return false;
  return image.crc32 == calculateCrc32(image);
}

bool FlightParameterDatabase::validatePayload(const FlightParameterPayload &payload)
{
  constexpr uint32_t REQUIRED_FIELDS = FlightParameterField::AIRFRAME | FlightParameterField::PWM |
                                       FlightParameterField::ATTITUDE_GAINS;
  if ((payload.valid_fields & REQUIRED_FIELDS) != REQUIRED_FIELDS) return false;
  if (payload.motor_count == 0U || payload.motor_count > MAX_THRUSTER_NUM) return false;
  if (payload.uav_model < FlightControlUavModel::DRONE) return false;

  const ThrusterPwmInfo &pwm = payload.pwm;
  if (!finite(pwm.min_pwm) || !finite(pwm.max_pwm) || !finite(pwm.min_thrust) || !finite(pwm.force_landing_thrust))
    return false;
  if (pwm.min_pwm < 0.0f || pwm.max_pwm <= pwm.min_pwm || pwm.max_pwm > ThrusterConstants::MAX_PWM) return false;
  if (pwm.motor_info_count == 0U || pwm.motor_info_count > MAX_THRUSTER_MOTOR_INFO_NUM) return false;
  for (size_t i = 0; i < pwm.motor_info_count; ++i)
  {
    if (!finite(pwm.motor_info[i].voltage) || !finite(pwm.motor_info[i].max_thrust) ||
        pwm.motor_info[i].voltage <= 0.0f || pwm.motor_info[i].max_thrust <= 0.0f)
      return false;
    for (float coefficient : pwm.motor_info[i].polynominal)
      if (!finite(coefficient)) return false;
  }

  const size_t gain_count = payload.attitude_gains.motors_count;
  if (gain_count != 1U && gain_count != payload.motor_count) return false;
  if (gain_count == 1U && (payload.valid_fields & FlightParameterField::TORQUE_ALLOCATION) == 0U) return false;

  if ((payload.valid_fields & FlightParameterField::P_MATRIX) != 0U &&
      payload.p_matrix.pseudo_inverse_count != payload.motor_count)
    return false;
  if ((payload.valid_fields & FlightParameterField::TORQUE_ALLOCATION) != 0U &&
      payload.torque_allocation.rows_count != payload.motor_count)
    return false;
  if ((payload.valid_fields & FlightParameterField::POSITION_CONTROL) != 0U &&
      payload.position_control.motor_count != payload.motor_count)
    return false;

  return true;
}
