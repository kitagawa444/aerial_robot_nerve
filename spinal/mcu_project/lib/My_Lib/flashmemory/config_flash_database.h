#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ImuDriver
{
enum : uint8_t
{
  DISABLED = 0U,
  MPU9250 = 1U,
  ICM20948 = 2U,
};
}

namespace Uart3Driver
{
enum : uint8_t
{
  DISABLED = 0U,
  GPS = 1U,
  CRSF = 2U,
};
}

namespace ServoDriver
{
enum : uint8_t
{
  DISABLED = 0U,
  DRIVER_DYNAMIXEL = 1U,
  DRIVER_KONDO = 2U,
};
}

namespace MotorOutputDriver
{
enum : uint8_t
{
  DISABLED = 0U,
  DRIVER_PWM = 1U,
  DRIVER_DSHOT = 2U,
};
}

struct ConfigFlashSupport
{
  bool imu_mpu9250{ false };
  bool imu_icm20948{ false };
  bool barometer{ false };
  bool uart3_gps{ false };
  bool uart3_crsf{ false };
  bool servo_dynamixel{ false };
  bool servo_kondo{ false };
  bool motor_pwm{ false };
  bool motor_dshot{ false };
  bool attitude_estimation{ false };
  bool height_estimation{ false };
  bool position_estimation{ false };
  bool flight_control{ false };
};

// Every field in this payload is a boot-time choice. The application image
// determines which choices are available; Config Flash selects among them.
struct ConfigFlashPayload
{
  uint8_t imu_driver{ ImuDriver::ICM20948 };
  uint8_t barometer_enabled{ 1U };
  uint8_t uart3_driver{ Uart3Driver::CRSF };
  uint8_t servo_driver{ ServoDriver::DRIVER_DYNAMIXEL };
  uint8_t attitude_estimation_enabled{ 1U };
  uint8_t height_estimation_enabled{ 1U };
  uint8_t position_estimation_enabled{ 1U };
  uint8_t flight_control_enabled{ 1U };
  uint8_t motor_output_driver{ MotorOutputDriver::DRIVER_DSHOT };
  uint8_t reserved[7]{};
};

struct ConfigFlashImage
{
  uint32_t magic{ 0U };
  uint16_t schema_version{ 0U };
  uint16_t image_size{ 0U };
  uint32_t generation{ 0U };
  ConfigFlashPayload payload{};
  uint32_t crc32{ 0U };
};

static_assert(std::is_trivially_copyable<ConfigFlashImage>::value,
              "Config Flash image must be safe for raw flash storage");
static_assert(sizeof(ConfigFlashImage) == 32U, "Config Flash image must occupy one STM32H7 flash word");

class ConfigFlashDatabase
{
public:
  static constexpr uint32_t MAGIC = 0x43464731U;  // "CFG1"
  static constexpr uint16_t SCHEMA_VERSION = 2U;
  static constexpr int8_t SLOT_NONE = -1;
  static constexpr int8_t SLOT_A = 0;
  static constexpr int8_t SLOT_B = 1;

  void load(const ConfigFlashImage &slot_a, const ConfigFlashImage &slot_b);
  void reload(const ConfigFlashImage &slot_a, const ConfigFlashImage &slot_b);
  void loadDefaults(const ConfigFlashPayload &defaults = ConfigFlashPayload{});
  bool stageConfiguration(const ConfigFlashPayload &payload, const ConfigFlashSupport &support);
  bool stageUart3Driver(uint8_t driver, bool gps_capable, bool crsf_capable);
  bool prepareCommit(ConfigFlashImage &image, int8_t &target_slot) const;
  bool acceptCommit(const ConfigFlashImage &image, int8_t slot);

  bool valid() const { return valid_; }
  bool dirty() const { return dirty_; }
  bool rebootRequired() const;
  const ConfigFlashPayload &activeConfiguration() const { return applied_; }
  const ConfigFlashPayload &pendingConfiguration() const { return staged_; }
  uint8_t activeUart3Driver() const { return applied_.uart3_driver; }
  uint8_t pendingUart3Driver() const { return staged_.uart3_driver; }
  void setAppliedConfiguration(const ConfigFlashPayload &payload) { applied_ = payload; }
  void setAppliedUart3Driver(uint8_t driver);
  int8_t activeSlot() const { return active_slot_; }
  uint16_t schemaVersion() const { return SCHEMA_VERSION; }
  uint32_t generation() const { return valid_ ? active_.generation : 0U; }
  uint32_t crc32() const { return valid_ ? active_.crc32 : 0U; }

  static uint32_t calculateCrc32(const ConfigFlashImage &image);
  static bool validateImage(const ConfigFlashImage &image);
  static bool validateConfiguration(const ConfigFlashPayload &payload, const ConfigFlashSupport &support);
  static bool validateDependencies(const ConfigFlashPayload &payload);
  static bool validateDriver(uint8_t driver, bool gps_capable, bool crsf_capable);
  static bool generationNewer(uint32_t lhs, uint32_t rhs);

private:
  ConfigFlashImage active_{};
  ConfigFlashPayload staged_{};
  ConfigFlashPayload applied_{};
  int8_t active_slot_{ SLOT_NONE };
  bool valid_{ false };
  bool dirty_{ false };

  void select_(const ConfigFlashImage &slot_a, const ConfigFlashImage &slot_b, bool update_applied);
  void refreshDirty_();
};
