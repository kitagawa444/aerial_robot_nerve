#include "flight_control/position/position_control.h"

#include <cmath>

bool PositionController::configure(const PositionControlConfig &config)
{
  if (config.motor_count == 0U || config.motor_count > MAX_FLIGHT_CONTROL_MOTOR_NUM ||
      config.max_horizontal_acceleration <= 0.0f || config.max_vertical_acceleration <= 0.0f ||
      config.max_tilt_angle <= 0.0f || config.setpoint_timeout_ms == 0U ||
      !std::isfinite(config.start_roll_pitch_integration_height) || config.start_roll_pitch_integration_height < 0.0f ||
      !std::isfinite(config.landing_err_z) || config.landing_err_z >= 0.0f ||
      !std::isfinite(config.safe_landing_height) || config.safe_landing_height < 0.0f ||
      (config.use_lqi_gains && !config.yaw_rate_feedback_on_spinal))
  {
    configured_ = false;
    return false;
  }

  if (!std::isfinite(config.rc_max_horizontal_velocity) || config.rc_max_horizontal_velocity <= 0.0f ||
      !std::isfinite(config.rc_max_vertical_velocity) || config.rc_max_vertical_velocity <= 0.0f ||
      !std::isfinite(config.rc_max_yaw_rate) || config.rc_max_yaw_rate <= 0.0f || !std::isfinite(config.rc_deadzone) ||
      config.rc_deadzone < 0.0f || config.rc_deadzone >= 1.0f || config.rc_timeout_ms == 0U)
  {
    configured_ = false;
    return false;
  }

  for (size_t motor = 0; motor < config.motor_count; ++motor)
  {
    if (!std::isfinite(config.vertical_acceleration_to_thrust[motor]) ||
        (!config.use_lqi_gains && config.vertical_acceleration_to_thrust[motor] <= 0.0f))
    {
      configured_ = false;
      return false;
    }
    if (config.use_lqi_gains && (!std::isfinite(config.yaw_acceleration_to_thrust[motor]) ||
                                 !std::isfinite(config.z_p_gain[motor]) || !std::isfinite(config.z_i_gain[motor]) ||
                                 !std::isfinite(config.z_d_gain[motor]) || !std::isfinite(config.yaw_p_gain[motor]) ||
                                 !std::isfinite(config.yaw_i_gain[motor]) || !std::isfinite(config.yaw_d_gain[motor])))
    {
      configured_ = false;
      return false;
    }
  }

  if (configured_ && sameConfig_(config)) return true;

  const bool reset_state = !configured_ || config_.motor_count != config.motor_count ||
                           config_.use_lqi_gains != config.use_lqi_gains;
  config_ = config;
  configured_ = true;
  if (reset_state)
  {
    reset();
  }
  else
  {
    for (size_t axis = 0; axis < 3; ++axis)
    {
      position_integral_[axis] = clamp_(position_integral_[axis], -config_.integral_limit[axis],
                                        config_.integral_limit[axis]);
    }
    yaw_integral_ = clamp_(yaw_integral_, -config_.yaw_limit_err_i, config_.yaw_limit_err_i);
  }
  return true;
}

bool PositionController::sameConfig_(const PositionControlConfig &config) const
{
  if (config_.motor_count != config.motor_count || config_.use_lqi_gains != config.use_lqi_gains ||
      config_.yaw_rate_feedback_on_spinal != config.yaw_rate_feedback_on_spinal || config_.yaw_p != config.yaw_p ||
      config_.yaw_i != config.yaw_i || config_.yaw_limit_sum != config.yaw_limit_sum ||
      config_.yaw_limit_err_p != config.yaw_limit_err_p || config_.yaw_limit_err_i != config.yaw_limit_err_i ||
      config_.yaw_limit_err_d != config.yaw_limit_err_d ||
      config_.start_roll_pitch_integration_height != config.start_roll_pitch_integration_height ||
      config_.landing_err_z != config.landing_err_z || config_.safe_landing_height != config.safe_landing_height ||
      config_.max_horizontal_acceleration != config.max_horizontal_acceleration ||
      config_.max_vertical_acceleration != config.max_vertical_acceleration ||
      config_.max_tilt_angle != config.max_tilt_angle || config_.setpoint_timeout_ms != config.setpoint_timeout_ms)
  {
    return false;
  }
  if (config_.rc_max_horizontal_velocity != config.rc_max_horizontal_velocity ||
      config_.rc_max_vertical_velocity != config.rc_max_vertical_velocity ||
      config_.rc_max_yaw_rate != config.rc_max_yaw_rate || config_.rc_deadzone != config.rc_deadzone ||
      config_.rc_timeout_ms != config.rc_timeout_ms)
  {
    return false;
  }

  for (size_t axis = 0; axis < 3; ++axis)
  {
    if (config_.position_p[axis] != config.position_p[axis] || config_.position_i[axis] != config.position_i[axis] ||
        config_.velocity_d[axis] != config.velocity_d[axis] || config_.limit_sum[axis] != config.limit_sum[axis] ||
        config_.limit_p[axis] != config.limit_p[axis] || config_.limit_i[axis] != config.limit_i[axis] ||
        config_.limit_d[axis] != config.limit_d[axis] || config_.limit_err_p[axis] != config.limit_err_p[axis] ||
        config_.limit_err_d[axis] != config.limit_err_d[axis] ||
        config_.integral_limit[axis] != config.integral_limit[axis])
    {
      return false;
    }
  }
  for (size_t motor = 0; motor < config.motor_count; ++motor)
  {
    if (config_.vertical_acceleration_to_thrust[motor] != config.vertical_acceleration_to_thrust[motor] ||
        config_.yaw_acceleration_to_thrust[motor] != config.yaw_acceleration_to_thrust[motor] ||
        config_.z_p_gain[motor] != config.z_p_gain[motor] || config_.z_i_gain[motor] != config.z_i_gain[motor] ||
        config_.z_d_gain[motor] != config.z_d_gain[motor] || config_.yaw_p_gain[motor] != config.yaw_p_gain[motor] ||
        config_.yaw_i_gain[motor] != config.yaw_i_gain[motor] || config_.yaw_d_gain[motor] != config.yaw_d_gain[motor])
    {
      return false;
    }
  }
  return true;
}

void PositionController::reset()
{
  position_integral_.zero();
  rc_position_offset_.zero();
  rc_input_ = PositionControlRcInput();
  yaw_integral_ = 0.0f;
  rc_yaw_offset_ = 0.0f;
  has_setpoint_ = false;
  setpoint_timestamp_ms_ = 0U;
  update_timestamp_ms_ = 0U;
  rc_input_timestamp_ms_ = 0U;
  previous_manual_control_allowed_ = false;
}

void PositionController::setEnabled(bool enabled)
{
  enabled_ = enabled;
  if (!enabled_) reset();
}

void PositionController::setSetpoint(const PositionControlSetpoint &setpoint, uint32_t timestamp_ms)
{
  if (!setpoint.active)
  {
    rc_position_offset_.zero();
    rc_yaw_offset_ = 0.0f;
  }
  else if (previous_manual_control_allowed_ && !setpoint.manual_control_allowed)
  {
    // Preserve the horizontal/yaw location reached by RC intervention during
    // landing, but remove altitude intervention so the navigation landing
    // height remains authoritative.
    rc_position_offset_.z = 0.0f;
  }
  setpoint_ = setpoint;
  previous_manual_control_allowed_ = setpoint.manual_control_allowed;
  setpoint_timestamp_ms_ = timestamp_ms;
  has_setpoint_ = true;
}

void PositionController::setRcInput(const PositionControlRcInput &input, uint32_t timestamp_ms)
{
  rc_input_ = input;
  rc_input_timestamp_ms_ = timestamp_ms;
}

bool PositionController::ready(const Eskf15 &estimator, uint32_t now_ms) const
{
  return ready(estimator.state(), estimator.validity(), now_ms);
}

bool PositionController::ready(const Eskf15::State &state, uint8_t validity, uint32_t now_ms) const
{
  (void)state;
  const uint8_t required_validity = Eskf15::HORIZONTAL_POSITION_VALID | Eskf15::VERTICAL_POSITION_VALID |
                                    Eskf15::VELOCITY_VALID | Eskf15::ATTITUDE_VALID;
  return configured_ && enabled_ && has_setpoint_ && now_ms - setpoint_timestamp_ms_ <= config_.setpoint_timeout_ms &&
         (validity & required_validity) == required_validity;
}

bool PositionController::shouldEnableAttitudeIntegration(const Eskf15::State &state) const
{
  return active() && state.position.z - setpoint_.initial_height > config_.start_roll_pitch_integration_height;
}

bool PositionController::update(const Eskf15 &estimator, uint32_t now_ms, FlightControlFourAxisCommand &command)
{
  return update(estimator.state(), estimator.validity(), now_ms, command);
}

bool PositionController::update(const Eskf15::State &state, uint8_t validity, uint32_t now_ms,
                                FlightControlFourAxisCommand &command)
{
  if (!ready(state, validity, now_ms)) return false;

  float dt = 0.0f;
  if (update_timestamp_ms_ != 0U) dt = static_cast<float>(now_ms - update_timestamp_ms_) * 0.001f;
  update_timestamp_ms_ = now_ms;
  if (!std::isfinite(dt) || dt < 0.0f || dt > 0.1f) dt = 0.0f;

  ap::Vector3f rc_velocity_world;
  float rc_yaw_rate = 0.0f;
  const bool rc_fresh = rc_input_.connected && now_ms - rc_input_timestamp_ms_ <= config_.rc_timeout_ms;
  if (setpoint_.manual_control_allowed && rc_fresh)
  {
    const float lateral = applyDeadzone_(rc_input_.lateral, config_.rc_deadzone) * config_.rc_max_horizontal_velocity;
    const float forward = applyDeadzone_(rc_input_.forward, config_.rc_deadzone) * config_.rc_max_horizontal_velocity;
    const float current_yaw = state.attitude.get_euler_yaw();
    const float cos_yaw = std::cos(current_yaw);
    const float sin_yaw = std::sin(current_yaw);
    rc_velocity_world.x = cos_yaw * forward - sin_yaw * lateral;
    rc_velocity_world.y = sin_yaw * forward + cos_yaw * lateral;
    rc_velocity_world.z = applyDeadzone_(rc_input_.vertical, config_.rc_deadzone) * config_.rc_max_vertical_velocity;
    rc_yaw_rate = applyDeadzone_(rc_input_.yaw, config_.rc_deadzone) * config_.rc_max_yaw_rate;
    rc_position_offset_ += rc_velocity_world * dt;
    rc_yaw_offset_ = wrapPi_(rc_yaw_offset_ + rc_yaw_rate * dt);
  }

  const ap::Vector3f effective_position = setpoint_.position + rc_position_offset_;
  const ap::Vector3f effective_velocity = setpoint_.velocity + rc_velocity_world;
  const float effective_yaw = wrapPi_(setpoint_.yaw + rc_yaw_offset_);
  const ap::Vector3f position_error = effective_position - state.position;
  const ap::Vector3f velocity_error = effective_velocity - state.velocity;

  ap::Vector3f acceleration_command = setpoint_.acceleration;
  for (size_t axis = 0; axis < 2; ++axis)
  {
    float controlled_position_error = position_error[axis];
    float controlled_velocity_error = velocity_error[axis];
    if (setpoint_.horizontal_control_mode == VELOCITY_CONTROL_MODE)
    {
      controlled_position_error = 0.0f;
    }
    else if (setpoint_.horizontal_control_mode == ACCELERATION_CONTROL_MODE)
    {
      controlled_position_error = 0.0f;
      controlled_velocity_error = 0.0f;
    }
    acceleration_command[axis] = updateAxis_(axis, controlled_position_error, controlled_velocity_error,
                                             setpoint_.acceleration[axis], dt);
  }

  if (config_.use_lqi_gains)
  {
    const float current_yaw = state.attitude.get_euler_yaw();
    const float cos_yaw = std::cos(current_yaw);
    const float sin_yaw = std::sin(current_yaw);
    const float acceleration_forward = cos_yaw * acceleration_command.x + sin_yaw * acceleration_command.y;
    const float acceleration_left = -sin_yaw * acceleration_command.x + cos_yaw * acceleration_command.y;

    command.angles[FlightControlAxis::X] = -acceleration_left / GRAVITY_MPS2;
    command.angles[FlightControlAxis::Y] = acceleration_forward / GRAVITY_MPS2;

    float z_position_error = position_error.z;
    float z_velocity_error = velocity_error.z;
    float z_dt = dt;
    if (setpoint_.landing)
    {
      if (-z_position_error > config_.safe_landing_height)
      {
        z_position_error = config_.landing_err_z;
        if (state.velocity.z < config_.landing_err_z) z_dt = 0.0f;
      }
      else
      {
        // The original controller disables only the Z P-term in the final
        // landing phase; velocity damping and the accumulated hover thrust
        // remain active.
        z_position_error = 0.0f;
      }
    }
    updateLqiThrust_(z_position_error, z_velocity_error, setpoint_.acceleration.z, z_dt, command);

    const float yaw_error = wrapPi_(effective_yaw - current_yaw);
    // The measured yaw-rate term remains in the high-rate attitude controller,
    // matching the original PC/spinal split when need_d_control is false.
    const float yaw_rate_error = setpoint_.yaw_rate + rc_yaw_rate;
    command.angles[FlightControlAxis::Z] = updateLqiYaw_(yaw_error, yaw_rate_error, setpoint_.yaw_acceleration, dt);
    return true;
  }

  acceleration_command.z = updateAxis_(2U, position_error.z, velocity_error.z, setpoint_.acceleration.z, dt);

  const float horizontal_norm = std::sqrt(acceleration_command.x * acceleration_command.x +
                                          acceleration_command.y * acceleration_command.y);
  if (horizontal_norm > config_.max_horizontal_acceleration)
  {
    const float scale = config_.max_horizontal_acceleration / horizontal_norm;
    acceleration_command.x *= scale;
    acceleration_command.y *= scale;
  }
  acceleration_command.z = clamp_(acceleration_command.z, -config_.max_vertical_acceleration,
                                  config_.max_vertical_acceleration);
  acceleration_command.z += GRAVITY_MPS2;

  const float cos_yaw = std::cos(effective_yaw);
  const float sin_yaw = std::sin(effective_yaw);
  const float acceleration_forward = cos_yaw * acceleration_command.x + sin_yaw * acceleration_command.y;
  const float acceleration_left = -sin_yaw * acceleration_command.x + cos_yaw * acceleration_command.y;
  const float vertical_reference = std::fmax(acceleration_command.z, 0.1f);

  command.angles[FlightControlAxis::X] = clamp_(
      std::atan2(-acceleration_left,
                 std::sqrt(acceleration_forward * acceleration_forward + vertical_reference * vertical_reference)),
      -config_.max_tilt_angle, config_.max_tilt_angle);
  command.angles[FlightControlAxis::Y] = clamp_(std::atan2(acceleration_forward, vertical_reference),
                                                -config_.max_tilt_angle, config_.max_tilt_angle);

  const float current_yaw = state.attitude.get_euler_yaw();
  const float yaw_error = wrapPi_(effective_yaw - current_yaw);
  yaw_integral_ = clamp_(yaw_integral_ + yaw_error * dt, -1.0f, 1.0f);
  command.angles[FlightControlAxis::Z] = config_.yaw_p * yaw_error + config_.yaw_i * yaw_integral_ +
                                         setpoint_.yaw_rate + rc_yaw_rate;

  const float total_acceleration = std::sqrt(acceleration_command.x * acceleration_command.x +
                                             acceleration_command.y * acceleration_command.y +
                                             acceleration_command.z * acceleration_command.z);
  command.base_thrust_count = config_.motor_count;
  for (size_t motor = 0; motor < config_.motor_count; ++motor)
  {
    command.base_thrust[motor] = config_.vertical_acceleration_to_thrust[motor] * total_acceleration;
  }
  return true;
}

float PositionController::updateAxis_(size_t axis, float position_error, float velocity_error, float feedforward,
                                      float dt)
{
  const float error_p = clamp_(position_error, -config_.limit_err_p[axis], config_.limit_err_p[axis]);
  const float error_d = clamp_(velocity_error, -config_.limit_err_d[axis], config_.limit_err_d[axis]);
  position_integral_[axis] = clamp_(position_integral_[axis] + error_p * dt, -config_.integral_limit[axis],
                                    config_.integral_limit[axis]);

  const float p_term = clamp_(config_.position_p[axis] * error_p, -config_.limit_p[axis], config_.limit_p[axis]);
  const float i_term = clamp_(config_.position_i[axis] * position_integral_[axis], -config_.limit_i[axis],
                              config_.limit_i[axis]);
  const float d_term = clamp_(config_.velocity_d[axis] * error_d, -config_.limit_d[axis], config_.limit_d[axis]);
  return clamp_(p_term + i_term + d_term + feedforward, -config_.limit_sum[axis], config_.limit_sum[axis]);
}

void PositionController::updateLqiThrust_(float position_error, float velocity_error, float feedforward, float dt,
                                          FlightControlFourAxisCommand &command)
{
  const size_t axis = FlightControlAxis::Z;
  const float error_p = clamp_(position_error, -config_.limit_err_p[axis], config_.limit_err_p[axis]);
  const float error_d = clamp_(velocity_error, -config_.limit_err_d[axis], config_.limit_err_d[axis]);
  const float previous_integral = position_integral_[axis];
  position_integral_[axis] = clamp_(position_integral_[axis] + error_p * dt, -config_.integral_limit[axis],
                                    config_.integral_limit[axis]);
  if (position_integral_[axis] < 0.0f) position_integral_[axis] = 0.0f;

  command.base_thrust_count = config_.motor_count;
  float max_term = 0.0f;
  for (size_t motor = 0; motor < config_.motor_count; ++motor)
  {
    command.base_thrust[motor] = config_.z_p_gain[motor] * error_p +
                                 config_.z_i_gain[motor] * position_integral_[axis] +
                                 config_.z_d_gain[motor] * error_d +
                                 config_.vertical_acceleration_to_thrust[motor] * feedforward;
    max_term = std::fmax(max_term, std::fabs(command.base_thrust[motor]));
  }

  if (max_term > config_.limit_sum[axis])
  {
    position_integral_[axis] = previous_integral;
    const float scale = config_.limit_sum[axis] / max_term;
    for (size_t motor = 0; motor < config_.motor_count; ++motor) command.base_thrust[motor] *= scale;
  }
}

float PositionController::updateLqiYaw_(float yaw_error, float yaw_rate_error, float yaw_acceleration, float dt)
{
  const float error_p = clamp_(yaw_error, -config_.yaw_limit_err_p, config_.yaw_limit_err_p);
  const float error_d = clamp_(yaw_rate_error, -config_.yaw_limit_err_d, config_.yaw_limit_err_d);
  const float previous_integral = yaw_integral_;
  yaw_integral_ = clamp_(yaw_integral_ + error_p * dt, -config_.yaw_limit_err_i, config_.yaw_limit_err_i);

  float motor_terms[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float max_term = 0.0f;
  for (size_t motor = 0; motor < config_.motor_count; ++motor)
  {
    motor_terms[motor] = config_.yaw_p_gain[motor] * error_p + config_.yaw_i_gain[motor] * yaw_integral_ +
                         config_.yaw_d_gain[motor] * error_d +
                         config_.yaw_acceleration_to_thrust[motor] * yaw_acceleration;
    max_term = std::fmax(max_term, std::fabs(motor_terms[motor]));
  }

  if (max_term > config_.yaw_limit_sum)
  {
    yaw_integral_ = previous_integral;
    const float scale = config_.yaw_limit_sum / max_term;
    for (size_t motor = 0; motor < config_.motor_count; ++motor) motor_terms[motor] *= scale;
  }

  float candidate = 0.0f;
  float max_yaw_d_gain = 0.0f;
  for (size_t motor = 0; motor < config_.motor_count; ++motor)
  {
    if (config_.yaw_d_gain[motor] > max_yaw_d_gain)
    {
      max_yaw_d_gain = config_.yaw_d_gain[motor];
      candidate = motor_terms[motor];
    }
  }
  return candidate;
}

float PositionController::clamp_(float value, float lower, float upper)
{
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

float PositionController::applyDeadzone_(float value, float deadzone)
{
  value = clamp_(value, -1.0f, 1.0f);
  const float magnitude = std::fabs(value);
  if (magnitude <= deadzone) return 0.0f;
  const float scaled = (magnitude - deadzone) / (1.0f - deadzone);
  return value < 0.0f ? -scaled : scaled;
}

float PositionController::wrapPi_(float angle)
{
  constexpr float PI = 3.14159265358979323846f;
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}
