#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "flight_control/health/health_types.h"
#include "flight_control/position/position_control.h"
#include "flight_control/flight_control_types.h"
#include "thruster/thruster_types.h"

namespace FlightParameterField
{
enum : uint32_t
{
  NONE = 0U,
  AIRFRAME = 1U << 0,
  PWM = 1U << 1,
  ATTITUDE_GAINS = 1U << 2,
  P_MATRIX = 1U << 3,
  TORQUE_ALLOCATION = 1U << 4,
  OFFSET_ROTATION = 1U << 5,
  POSITION_CONTROL = 1U << 6,
  HEALTH = 1U << 7,
};
}

struct FlightParameterPayload
{
  uint32_t valid_fields{ FlightParameterField::NONE };
  uint8_t motor_count{ 0U };
  int8_t uav_model{ -1 };
  uint8_t gimbal_dof{ 0U };
  uint8_t position_control_enabled{ 0U };
  uint8_t attitude_control_enabled{ 1U };
  uint8_t reserved[3]{};
  ThrusterPwmInfo pwm{};
  FlightControlRpyTerms attitude_gains{};
  FlightControlPMatrixPseudoInverseWithInertia p_matrix{};
  FlightControlTorqueAllocationMatrixInv torque_allocation{};
  FlightControlDesireCoord offset_rotation{};
  PositionControlConfig position_control{};
  HealthManagerConfig health{};
};

struct FlightParameterImage
{
  uint32_t magic{ 0U };
  uint16_t schema_version{ 0U };
  uint16_t image_size{ 0U };
  uint32_t generation{ 0U };
  uint32_t crc32{ 0U };
  FlightParameterPayload payload{};
};

static_assert(std::is_trivially_copyable<FlightParameterImage>::value,
              "Flight parameter image must be safe for raw flash storage");
static_assert(sizeof(FlightParameterImage) <= 0xFFFFU, "Flight parameter image exceeds its encoded size field");

class FlightParameterDatabase
{
public:
  static constexpr uint32_t MAGIC = 0x50444231U;  // "PDB1"
  static constexpr uint16_t SCHEMA_VERSION = 1U;

  void load();
  bool stage(const FlightParameterPayload &payload);
  bool prepareCommit();
  void markCommitFailed()
  {
    valid_ = false;
    dirty_ = true;
  }

  bool valid() const { return valid_; }
  bool dirty() const { return dirty_; }
  uint16_t schemaVersion() const { return SCHEMA_VERSION; }
  uint32_t generation() const { return valid_ ? image_.generation : 0U; }
  uint32_t crc32() const { return valid_ ? image_.crc32 : 0U; }
  uint32_t validFields() const { return staged_.valid_fields; }
  const FlightParameterPayload &payload() const { return staged_; }

  void *storageData() { return &image_; }
  const void *storageData() const { return &image_; }
  size_t storageSize() const { return sizeof(image_); }

  static uint32_t calculateCrc32(const FlightParameterImage &image);
  static bool validateImage(const FlightParameterImage &image);
  static bool validatePayload(const FlightParameterPayload &payload);

private:
  FlightParameterImage image_{};
  FlightParameterPayload staged_{};
  bool valid_{ false };
  bool dirty_{ false };
};
