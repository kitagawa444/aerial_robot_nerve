#pragma once

#include <cstddef>
#include <cstdint>

#include "flight_control/flight_control_types.h"
#include "state_estimate/eskf/eskf15.h"

struct PositionControlConfig
{
  float position_p[3]{ 1.5f, 1.5f, 2.0f };
  float position_i[3]{ 0.0f, 0.0f, 0.2f };
  float velocity_d[3]{ 1.0f, 1.0f, 1.2f };
  float limit_sum[3]{ 1.0e6f, 1.0e6f, 1.0e6f };
  float limit_p[3]{ 1.0e6f, 1.0e6f, 1.0e6f };
  float limit_i[3]{ 1.0e6f, 1.0e6f, 1.0e6f };
  float limit_d[3]{ 1.0e6f, 1.0e6f, 1.0e6f };
  float limit_err_p[3]{ 1.0e6f, 1.0e6f, 1.0e6f };
  float limit_err_d[3]{ 1.0e6f, 1.0e6f, 1.0e6f };
  float yaw_p{ 0.5f };
  float yaw_i{ 0.05f };
  float yaw_limit_sum{ 1.0e6f };
  float yaw_limit_err_p{ 1.0e6f };
  float yaw_limit_err_i{ 1.0e6f };
  float yaw_limit_err_d{ 1.0e6f };
  float max_horizontal_acceleration{ 4.0f };
  float max_vertical_acceleration{ 5.0f };
  float max_tilt_angle{ 0.7f };
  float integral_limit[3]{ 2.0f, 2.0f, 2.0f };
  float vertical_acceleration_to_thrust[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float yaw_acceleration_to_thrust[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float z_p_gain[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float z_i_gain[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float z_d_gain[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float yaw_p_gain[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float yaw_i_gain[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float yaw_d_gain[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  size_t motor_count{ 0U };
  bool use_lqi_gains{ false };
  bool yaw_rate_feedback_on_spinal{ true };
  float start_roll_pitch_integration_height{ 0.01f };
  float landing_err_z{ -0.5f };
  float safe_landing_height{ 0.1f };
  uint32_t setpoint_timeout_ms{ 500U };
  float rc_max_horizontal_velocity{ 0.5f };
  float rc_max_vertical_velocity{ 0.5f };
  float rc_max_yaw_rate{ 0.1f };
  float rc_deadzone{ 0.2f };
  uint32_t rc_timeout_ms{ 100U };
};

struct PositionControlSetpoint
{
  ap::Vector3f position{};
  ap::Vector3f velocity{};
  ap::Vector3f acceleration{};
  float yaw{ 0.0f };
  float yaw_rate{ 0.0f };
  float yaw_acceleration{ 0.0f };
  float initial_height{ 0.0f };
  uint8_t horizontal_control_mode{ 0U };
  bool active{ false };
  bool manual_control_allowed{ false };
  bool landing{ false };
};

struct PositionControlRcInput
{
  float lateral{ 0.0f };
  float forward{ 0.0f };
  float vertical{ 0.0f };
  float yaw{ 0.0f };
  bool connected{ false };
};

class PositionController
{
public:
  PositionController() = default;

  bool configure(const PositionControlConfig &config);
  void reset();
  void setEnabled(bool enabled);
  bool enabled() const { return enabled_; }
  bool configured() const { return configured_; }
  bool active() const { return has_setpoint_ && setpoint_.active; }
  bool canReconfigureInFlight(const PositionControlConfig &config) const
  {
    return configured_ && config_.motor_count == config.motor_count && config_.use_lqi_gains == config.use_lqi_gains;
  }

  void setSetpoint(const PositionControlSetpoint &setpoint, uint32_t timestamp_ms);
  void setRcInput(const PositionControlRcInput &input, uint32_t timestamp_ms);
  bool ready(const Eskf15 &estimator, uint32_t now_ms) const;
  bool ready(const Eskf15::State &state, uint8_t validity, uint32_t now_ms) const;
  bool update(const Eskf15 &estimator, uint32_t now_ms, FlightControlFourAxisCommand &command);
  bool update(const Eskf15::State &state, uint8_t validity, uint32_t now_ms, FlightControlFourAxisCommand &command);
  bool shouldEnableAttitudeIntegration(const Eskf15::State &state) const;

  const PositionControlSetpoint &setpoint() const { return setpoint_; }
  const ap::Vector3f &positionIntegral() const { return position_integral_; }
  const ap::Vector3f &rcPositionOffset() const { return rc_position_offset_; }
  float rcYawOffset() const { return rc_yaw_offset_; }

private:
  static constexpr float GRAVITY_MPS2 = 9.80665f;
  static constexpr uint8_t POSITION_CONTROL_MODE = 0U;
  static constexpr uint8_t VELOCITY_CONTROL_MODE = 1U;
  static constexpr uint8_t ACCELERATION_CONTROL_MODE = 2U;

  PositionControlConfig config_{};
  PositionControlSetpoint setpoint_{};
  ap::Vector3f position_integral_{};
  ap::Vector3f rc_position_offset_{};
  PositionControlRcInput rc_input_{};
  float yaw_integral_{ 0.0f };
  float rc_yaw_offset_{ 0.0f };
  uint32_t setpoint_timestamp_ms_{ 0U };
  uint32_t rc_input_timestamp_ms_{ 0U };
  uint32_t update_timestamp_ms_{ 0U };
  bool configured_{ false };
  bool enabled_{ false };
  bool has_setpoint_{ false };
  bool previous_manual_control_allowed_{ false };

  bool sameConfig_(const PositionControlConfig &config) const;
  float updateAxis_(size_t axis, float position_error, float velocity_error, float feedforward, float dt);
  void updateLqiThrust_(float position_error, float velocity_error, float feedforward, float dt,
                        FlightControlFourAxisCommand &command);
  float updateLqiYaw_(float yaw_error, float yaw_rate_error, float yaw_acceleration, float dt);
  static float clamp_(float value, float lower, float upper);
  static float applyDeadzone_(float value, float deadzone);
  static float wrapPi_(float angle);
};
