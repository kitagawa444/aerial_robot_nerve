#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace aerial {
namespace vehicle {

constexpr uint16_t kProtocolVersion = 2U;
constexpr size_t kMaxPayloadSize = 512U;

enum class MessageId : uint16_t {
  HEARTBEAT = 1,
  IMU = 2,
  STATE_ESTIMATE = 3,
  FLIGHT_STATUS = 4,
  BATTERY_STATUS = 5,
  RC_STATUS = 6,
  RC_CHANNELS = 7,
  MOTOR_OUTPUTS = 8,
  EVENT = 9,
  APPLICATION_CAPABILITIES = 10,
  CONFIG_FLASH_STATUS = 11,
  FLIGHT_PARAMETER_STATUS = 12,
  FLIGHT_PARAMETER_CHUNK = 13,
  REBOOT_STATUS = 14,
  CONFIG_ACK = 15,
  BOOTLOADER_STATUS = 16,
  PROTOCOL_ACK = 17,

  NETWORK_HEARTBEAT = 100,
  FLIGHT_COMMAND = 101,
  EXTERNAL_STATE_MEASUREMENT = 102,
  POSITION_CONTROL_SETPOINT = 103,
  HEALTH_CONFIG = 104,
  PARAMETER_REQUEST = 105,
  CONFIG_FLASH_REQUEST = 106,
  REBOOT_REQUEST = 107,
  ROS_HEARTBEAT = 108,
  CONFIGURATION_CHUNK = 109,
  BOOTLOADER_REQUEST = 110,
};

enum class Source : uint8_t {
  UNKNOWN = 0,
  INTERNAL = 1,
  ROS2 = 2,
  MAVLINK = 3,
  RC = 4,
};

enum class Reliability : uint8_t {
  BEST_EFFORT = 0,
  RELIABLE = 1,
};

// Command authority is intentionally separate from the transport source. The
// supervisor records who may control the vehicle, while FrameHeader::source
// records which adapter delivered the frame.
enum class CommandSource : uint8_t {
  NONE = 0,
  ROS2 = 1,
  RC = 2,
  INTERNAL = 3,
  FAILSAFE = 4,
  MAVLINK = 5,
};

#pragma pack(push, 1)

struct FrameHeader {
  uint16_t protocol_version{kProtocolVersion};
  uint16_t message_id{0U};
  uint32_t sequence{0U};
  uint32_t request_id{0U};
  uint64_t timestamp_us{0U};
  uint8_t source{static_cast<uint8_t>(Source::UNKNOWN)};
  uint8_t reliability{static_cast<uint8_t>(Reliability::BEST_EFFORT)};
  uint16_t payload_size{0U};
};

struct Frame {
  FrameHeader header{};
  uint8_t payload[kMaxPayloadSize]{};
};

struct Heartbeat {
  uint32_t uptime_ms{0U};
  uint8_t link_state{0U};
  uint8_t reserved[3]{};
};

struct Imu {
  float acceleration[3]{};
  float angular_velocity[3]{};
  float magnetic_field[3]{};
  float attitude_xyzw[4]{};
};

struct StateEstimate {
  uint8_t validity{0U};
  uint8_t reserved[3]{};
  float position[3]{};
  float velocity[3]{};
  float acceleration[3]{};
  float attitude_xyzw[4]{};
  float angular_velocity[3]{};
  float accelerometer_bias[3]{};
  float gyroscope_bias[3]{};
  float covariance_diagonal[15]{};
  uint32_t rejected_measurements{0U};
};

struct HealthStatus {
  uint8_t state{0U};
  uint8_t reserved[3]{};
  uint32_t reason_mask{0U};
  uint32_t last_change_ms{0U};
};

struct HealthReport {
  HealthStatus overall{};
  HealthStatus flight{};
  HealthStatus link{};
  HealthStatus power{};
  HealthStatus sensor{};
  HealthStatus estimation{};
  HealthStatus actuator{};
  HealthStatus compute{};
  HealthStatus config{};
  uint8_t ros_link_state{0U};
  uint8_t network_link_state{0U};
  uint8_t rc_link_state{0U};
  uint8_t requested_action{0U};
  uint8_t action_source{0U};
  uint8_t reserved[3]{};
  uint32_t action_reason_mask{0U};
  uint32_t action_since_ms{0U};
  float battery_voltage{0.0F};
  float battery_compensated_voltage{0.0F};
  float battery_percentage{0.0F};
  uint8_t estimation_validity{0U};
  uint8_t reserved2[3]{};
  uint32_t rejected_measurements{0U};
};

struct FlightStatus {
  uint32_t sequence{0U};
  uint32_t command_sequence{0U};
  uint8_t arming_state{0U};
  uint8_t flight_phase{0U};
  uint8_t control_mode{0U};
  uint8_t user_intention{0U};
  uint8_t authority{0U};
  uint8_t failsafe{0U};
  uint8_t last_command{0U};
  uint8_t last_command_source{0U};
  uint8_t last_command_result{0U};
  uint8_t transition_reason{0U};
  uint8_t ready_to_arm{0U};
  uint8_t attitude_ready{0U};
  uint8_t position_ready{0U};
  uint8_t rc_connected{0U};
  uint8_t ros_link_state{0U};
  uint8_t network_link_state{0U};
  uint8_t rc_link_state{0U};
  uint8_t reserved{0U};
  HealthReport health{};
  float takeoff_reference_height{0.0F};
  float takeoff_target_height{0.0F};
};

struct BatteryStatus {
  float voltage{0.0F};
  float compensated_voltage{0.0F};
  float percentage{0.0F};
  float current{0.0F};
};

struct RcStatus {
  uint8_t connected{0U};
  uint8_t link_quality{0U};
  int8_t rssi_dbm{0};
  uint8_t reserved{0U};
};

struct RcChannels {
  uint16_t channels[16]{};
};

enum class ConfigurationKind : uint16_t {
  AIRFRAME = 1,
  GIMBAL_DOF = 2,
  FOUR_AXIS_COMMAND = 3,
  PWM_INFO = 4,
  ATTITUDE_GAINS = 5,
  P_MATRIX = 6,
  TORQUE_ALLOCATION = 7,
  OFFSET_ROTATION = 8,
  POSITION_CONTROL = 9,
  ATTITUDE_CONTROL_ENABLED = 10,
  POSITION_CONTROL_ENABLED = 11,
};

struct AirframeConfig {
  uint8_t motor_count{0U};
  int8_t uav_model{-1};
  uint8_t reserved[2]{};
};

constexpr size_t kChunkDataSize = 480U;

struct ConfigurationChunk {
  uint32_t update_id{0U};
  uint16_t kind{0U};
  uint16_t total_size{0U};
  uint16_t offset{0U};
  uint16_t chunk_size{0U};
  uint8_t data[kChunkDataSize]{};
};

struct ApplicationCapabilities {
  uint32_t capability_mask{0U};
  uint16_t config_schema_version{0U};
  uint16_t flight_parameter_schema_version{0U};
  uint8_t board{0U};
  uint8_t max_motor_count{0U};
  uint8_t reserved[2]{};
};

struct ConfigFlashSelection {
  uint8_t uart3_driver{0U};
  uint8_t imu_driver{0U};
  uint8_t barometer_enabled{0U};
  uint8_t servo_driver{0U};
  uint8_t attitude_estimation_enabled{0U};
  uint8_t height_estimation_enabled{0U};
  uint8_t position_estimation_enabled{0U};
  uint8_t flight_control_enabled{0U};
  uint8_t motor_output_driver{0U};
  uint8_t reserved[3]{};
};

struct ConfigFlashRequest {
  uint32_t request_id{0U};
  uint8_t command{0U};
  uint8_t reserved[3]{};
  ConfigFlashSelection selection{};
};

struct ConfigFlashStatus {
  uint32_t request_id{0U};
  uint32_t generation{0U};
  uint32_t crc32{0U};
  uint16_t schema_version{0U};
  int8_t active_slot{-1};
  uint8_t result{0U};
  uint8_t success{0U};
  uint8_t valid{0U};
  uint8_t dirty{0U};
  uint8_t reboot_required{0U};
  uint8_t persistent_storage{0U};
  uint8_t reserved[3]{};
  ConfigFlashSelection active{};
  ConfigFlashSelection pending{};
};

struct ParameterRequest {
  uint32_t request_id{0U};
  uint8_t command{0U};
  uint8_t reserved[3]{};
};

struct FlightParameterStatus {
  uint32_t request_id{0U};
  uint32_t generation{0U};
  uint32_t crc32{0U};
  uint32_t valid_fields{0U};
  uint16_t schema_version{0U};
  uint16_t payload_size{0U};
  uint8_t result{0U};
  uint8_t success{0U};
  uint8_t valid{0U};
  uint8_t dirty{0U};
  uint8_t applied{0U};
  uint8_t persistent_storage{0U};
  uint8_t reserved[2]{};
};

struct FlightParameterChunk {
  uint32_t request_id{0U};
  uint16_t total_size{0U};
  uint16_t offset{0U};
  uint16_t chunk_size{0U};
  uint8_t reserved[2]{};
  uint8_t data[kChunkDataSize]{};
};

struct RebootRequest {
  uint32_t request_id{0U};
};

struct RebootStatus {
  uint32_t request_id{0U};
  uint8_t success{0U};
  uint8_t result{0U};
  uint8_t reserved[2]{};
};

enum class ProtocolAckResult : uint8_t {
  ACCEPTED = 0,
  REJECTED = 1,
  INVALID_REQUEST = 2,
};

struct ProtocolAck {
  uint32_t request_id{0U};
  uint16_t message_id{0U};
  uint8_t request_source{static_cast<uint8_t>(Source::UNKNOWN)};
  uint8_t result{static_cast<uint8_t>(ProtocolAckResult::ACCEPTED)};
  uint8_t duplicate{0U};
  uint8_t reserved[3]{};
};

struct ConfigAck {
  uint8_t command{0U};
  uint8_t reserved[3]{};
};

struct FlightCommand {
  uint32_t command_id{0U};
  uint32_t issued_at_ms{0U};
  uint8_t command{0U};
  uint8_t source{static_cast<uint8_t>(CommandSource::NONE)};
  uint8_t target_system{0U};
  uint8_t target_component{0U};
};

struct ExternalStateMeasurement {
  uint8_t estimate_mode{0U};
  uint8_t field_mask{0U};
  uint8_t reserved[2]{};
  float position[3]{};
  float position_variance[3]{};
  float velocity[3]{};
  float velocity_variance[3]{};
  float attitude_xyzw[4]{};
  float attitude_variance[3]{};
  float angular_velocity[3]{};
};

struct PositionControlSetpoint {
  float position[3]{};
  float velocity[3]{};
  float acceleration[3]{};
  float yaw{0.0F};
  float yaw_rate{0.0F};
  float yaw_acceleration{0.0F};
  float initial_height{0.0F};
  uint8_t horizontal_control_mode{0U};
  uint8_t active{0U};
  uint8_t manual_control_allowed{0U};
  uint8_t landing{0U};
};

struct HealthConfig {
  uint8_t battery_cell_count{0U};
  uint8_t control_loop_miss_limit{0U};
  uint8_t reserved[2]{};
  float battery_low_percentage{0.0F};
  float battery_hysteresis_percentage{0.0F};
  float battery_high_cell_threshold{0.0F};
  float battery_resistance{0.0F};
  float battery_resistance_voltage_rate{0.0F};
  float battery_hovering_current{0.0F};
  uint32_t battery_debounce_ms{0U};
  uint32_t primary_imu_timeout_ms{0U};
  uint32_t control_loop_deadline_ms{0U};
};

constexpr size_t kFlightMotorCount = 10U;
constexpr size_t kPwmMotorInfoCount = 16U;

struct FourAxisConfiguration {
  float angles[3]{};
  float base_thrust[kFlightMotorCount]{};
  uint8_t base_thrust_count{0U};
  uint8_t reserved[3]{};
};

struct PwmMotorInfo {
  float voltage{0.0F};
  float max_thrust{0.0F};
  float polynomial[5]{};
};

struct PwmConfiguration {
  float min_pwm{0.0F};
  float max_pwm{0.0F};
  float min_thrust{0.0F};
  float force_landing_thrust{0.0F};
  uint8_t conversion_mode{0U};
  uint8_t motor_info_count{0U};
  uint8_t reserved[2]{};
  PwmMotorInfo motor_info[kPwmMotorInfoCount]{};
};

struct RpyGain {
  int16_t roll_p{0};
  int16_t roll_i{0};
  int16_t roll_d{0};
  int16_t pitch_p{0};
  int16_t pitch_i{0};
  int16_t pitch_d{0};
  int16_t yaw_d{0};
};

struct RpyGainsConfiguration {
  uint8_t motor_count{0U};
  uint8_t reserved[3]{};
  RpyGain motors[kFlightMotorCount]{};
};

struct PMatrixUnit {
  int16_t r{0};
  int16_t p{0};
  int16_t y{0};
};

struct PMatrixConfiguration {
  uint8_t motor_count{0U};
  uint8_t reserved[3]{};
  PMatrixUnit pseudo_inverse[kFlightMotorCount]{};
  int16_t inertia[6]{};
};

struct TorqueAllocationRow {
  int16_t x{0};
  int16_t y{0};
  int16_t z{0};
};

struct TorqueAllocationConfiguration {
  uint8_t row_count{0U};
  uint8_t reserved[3]{};
  TorqueAllocationRow rows[kFlightMotorCount]{};
};

struct OffsetRotationConfiguration {
  float roll{0.0F};
  float pitch{0.0F};
  float yaw{0.0F};
};

struct SupervisorConfiguration {
  float takeoff_height{0.0F};
  float takeoff_position_tolerance{0.0F};
  float takeoff_velocity_tolerance{0.0F};
  uint32_t takeoff_stable_time_ms{0U};
  float landing_speed{0.0F};
  float landed_height{0.0F};
  float landed_velocity{0.0F};
  uint32_t landed_stable_time_ms{0U};
  uint32_t rc_authority_timeout_ms{0U};
};

struct PositionControlConfiguration {
  float position_p[3]{};
  float position_i[3]{};
  float velocity_d[3]{};
  float limit_sum[3]{};
  float limit_p[3]{};
  float limit_i[3]{};
  float limit_d[3]{};
  float limit_err_p[3]{};
  float limit_err_d[3]{};
  float integral_limit[3]{};
  float yaw_p{0.0F};
  float yaw_i{0.0F};
  float yaw_limit_sum{0.0F};
  float yaw_limit_err_p{0.0F};
  float yaw_limit_err_i{0.0F};
  float yaw_limit_err_d{0.0F};
  float max_horizontal_acceleration{0.0F};
  float max_vertical_acceleration{0.0F};
  float max_tilt_angle{0.0F};
  float vertical_acceleration_to_thrust[kFlightMotorCount]{};
  float yaw_acceleration_to_thrust[kFlightMotorCount]{};
  float z_p_gain[kFlightMotorCount]{};
  float z_i_gain[kFlightMotorCount]{};
  float z_d_gain[kFlightMotorCount]{};
  float yaw_p_gain[kFlightMotorCount]{};
  float yaw_i_gain[kFlightMotorCount]{};
  float yaw_d_gain[kFlightMotorCount]{};
  uint8_t motor_count{0U};
  uint8_t use_lqi_gains{0U};
  uint8_t yaw_rate_feedback_on_spinal{0U};
  uint8_t reserved{0U};
  float start_roll_pitch_integration_height{0.0F};
  float landing_err_z{0.0F};
  float safe_landing_height{0.0F};
  uint32_t setpoint_timeout_ms{0U};
  float rc_max_horizontal_velocity{0.0F};
  float rc_max_vertical_velocity{0.0F};
  float rc_max_yaw_rate{0.0F};
  float rc_deadzone{0.0F};
  uint32_t rc_timeout_ms{0U};
  SupervisorConfiguration supervisor{};
};

struct FlightParameterSnapshot {
  uint32_t valid_fields{0U};
  uint8_t motor_count{0U};
  int8_t uav_model{-1};
  uint8_t gimbal_dof{0U};
  uint8_t position_control_enabled{0U};
  uint8_t attitude_control_enabled{0U};
  uint8_t reserved[3]{};
  PwmConfiguration pwm{};
  RpyGainsConfiguration attitude_gains{};
  PMatrixConfiguration p_matrix{};
  TorqueAllocationConfiguration torque_allocation{};
  OffsetRotationConfiguration offset_rotation{};
  PositionControlConfiguration position_control{};
  HealthConfig health{};
};

struct Event {
  uint32_t event_id{0U};
  uint32_t sequence{0U};
  uint32_t argument{0U};
  uint8_t severity{0U};
  uint8_t source{0U};
  uint8_t reserved[2]{};
};

#pragma pack(pop)

static_assert(sizeof(FourAxisConfiguration) == 56U,
              "Four-axis wire layout changed");
static_assert(sizeof(PwmConfiguration) == 468U, "PWM wire layout changed");
static_assert(sizeof(RpyGainsConfiguration) == 144U,
              "Attitude-gain wire layout changed");
static_assert(sizeof(PMatrixConfiguration) == 76U,
              "P-matrix wire layout changed");
static_assert(sizeof(TorqueAllocationConfiguration) == 64U,
              "Torque-allocation wire layout changed");
static_assert(sizeof(PositionControlConfiguration) == 552U,
              "Position-control wire layout changed");
static_assert(sizeof(FlightParameterSnapshot) == 1368U,
              "Flight-parameter snapshot wire layout changed");

template <typename T>
bool setPayload(Frame &frame, MessageId message_id, const T &value,
                uint32_t sequence, uint64_t timestamp_us, Source source,
                Reliability reliability, uint32_t request_id = 0U) {
  static_assert(std::is_trivially_copyable<T>::value,
                "Spinal Link payloads must be trivially copyable");
  static_assert(sizeof(T) <= kMaxPayloadSize,
                "Spinal Link payload exceeds frame capacity");

  frame.header.protocol_version = kProtocolVersion;
  frame.header.message_id = static_cast<uint16_t>(message_id);
  frame.header.sequence = sequence;
  frame.header.request_id = request_id;
  frame.header.timestamp_us = timestamp_us;
  frame.header.source = static_cast<uint8_t>(source);
  frame.header.reliability = static_cast<uint8_t>(reliability);
  frame.header.payload_size = static_cast<uint16_t>(sizeof(T));
  std::memcpy(frame.payload, &value, sizeof(T));
  return true;
}

template <typename T>
bool getPayload(const Frame &frame, MessageId expected_message_id, T &value) {
  static_assert(std::is_trivially_copyable<T>::value,
                "Spinal Link payloads must be trivially copyable");
  if (frame.header.protocol_version != kProtocolVersion ||
      frame.header.message_id != static_cast<uint16_t>(expected_message_id) ||
      frame.header.payload_size != sizeof(T)) {
    return false;
  }

  std::memcpy(&value, frame.payload, sizeof(T));
  return true;
}

inline bool validFrame(const Frame &frame) {
  return frame.header.protocol_version == kProtocolVersion &&
         frame.header.payload_size <= kMaxPayloadSize;
}

} // namespace vehicle
} // namespace aerial
