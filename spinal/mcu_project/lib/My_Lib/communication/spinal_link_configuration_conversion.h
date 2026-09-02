#pragma once

#include <algorithm>
#include <cstddef>

#include "communication/spinal_link_protocol.h"
#include "flashmemory/parameter_database.h"

namespace aerial {
namespace vehicle {
namespace conversion {

static_assert(MAX_FLIGHT_CONTROL_MOTOR_NUM == kFlightMotorCount,
              "Spinal Link motor count must be versioned before changing");
static_assert(
    MAX_THRUSTER_MOTOR_INFO_NUM == kPwmMotorInfoCount,
    "Spinal Link PWM table size must be versioned before changing");

inline FlightControlFourAxisCommand
toInternal(const FourAxisConfiguration &source) {
  FlightControlFourAxisCommand destination{};
  for (size_t axis = 0U; axis < 3U; ++axis)
    destination.angles[axis] = source.angles[axis];
  destination.base_thrust_count =
      std::min(static_cast<size_t>(source.base_thrust_count),
               static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0U; motor < destination.base_thrust_count; ++motor)
    destination.base_thrust[motor] = source.base_thrust[motor];
  return destination;
}

inline ThrusterPwmInfo toInternal(const PwmConfiguration &source) {
  ThrusterPwmInfo destination{};
  destination.min_pwm = source.min_pwm;
  destination.max_pwm = source.max_pwm;
  destination.min_thrust = source.min_thrust;
  destination.force_landing_thrust = source.force_landing_thrust;
  destination.pwm_conversion_mode = source.conversion_mode;
  destination.motor_info_count =
      std::min(static_cast<size_t>(source.motor_info_count),
               static_cast<size_t>(MAX_THRUSTER_MOTOR_INFO_NUM));
  for (size_t motor = 0U; motor < destination.motor_info_count; ++motor) {
    destination.motor_info[motor].voltage = source.motor_info[motor].voltage;
    destination.motor_info[motor].max_thrust =
        source.motor_info[motor].max_thrust;
    for (size_t coefficient = 0U; coefficient < 5U; ++coefficient)
      destination.motor_info[motor].polynominal[coefficient] =
          source.motor_info[motor].polynomial[coefficient];
  }
  return destination;
}

inline PwmConfiguration toWire(const ThrusterPwmInfo &source) {
  PwmConfiguration destination{};
  destination.min_pwm = source.min_pwm;
  destination.max_pwm = source.max_pwm;
  destination.min_thrust = source.min_thrust;
  destination.force_landing_thrust = source.force_landing_thrust;
  destination.conversion_mode = source.pwm_conversion_mode;
  destination.motor_info_count = static_cast<uint8_t>(std::min(
      source.motor_info_count, static_cast<size_t>(kPwmMotorInfoCount)));
  for (size_t motor = 0U; motor < destination.motor_info_count; ++motor) {
    destination.motor_info[motor].voltage = source.motor_info[motor].voltage;
    destination.motor_info[motor].max_thrust =
        source.motor_info[motor].max_thrust;
    for (size_t coefficient = 0U; coefficient < 5U; ++coefficient)
      destination.motor_info[motor].polynomial[coefficient] =
          source.motor_info[motor].polynominal[coefficient];
  }
  return destination;
}

inline FlightControlRpyTerms toInternal(const RpyGainsConfiguration &source) {
  FlightControlRpyTerms destination{};
  destination.motors_count =
      std::min(static_cast<size_t>(source.motor_count),
               static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0U; motor < destination.motors_count; ++motor) {
    destination.motors[motor].roll_p = source.motors[motor].roll_p;
    destination.motors[motor].roll_i = source.motors[motor].roll_i;
    destination.motors[motor].roll_d = source.motors[motor].roll_d;
    destination.motors[motor].pitch_p = source.motors[motor].pitch_p;
    destination.motors[motor].pitch_i = source.motors[motor].pitch_i;
    destination.motors[motor].pitch_d = source.motors[motor].pitch_d;
    destination.motors[motor].yaw_d = source.motors[motor].yaw_d;
  }
  return destination;
}

inline RpyGainsConfiguration toWire(const FlightControlRpyTerms &source) {
  RpyGainsConfiguration destination{};
  destination.motor_count = static_cast<uint8_t>(
      std::min(source.motors_count, static_cast<size_t>(kFlightMotorCount)));
  for (size_t motor = 0U; motor < destination.motor_count; ++motor) {
    destination.motors[motor].roll_p = source.motors[motor].roll_p;
    destination.motors[motor].roll_i = source.motors[motor].roll_i;
    destination.motors[motor].roll_d = source.motors[motor].roll_d;
    destination.motors[motor].pitch_p = source.motors[motor].pitch_p;
    destination.motors[motor].pitch_i = source.motors[motor].pitch_i;
    destination.motors[motor].pitch_d = source.motors[motor].pitch_d;
    destination.motors[motor].yaw_d = source.motors[motor].yaw_d;
  }
  return destination;
}

inline FlightControlPMatrixPseudoInverseWithInertia
toInternal(const PMatrixConfiguration &source) {
  FlightControlPMatrixPseudoInverseWithInertia destination{};
  destination.pseudo_inverse_count =
      std::min(static_cast<size_t>(source.motor_count),
               static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0U; motor < destination.pseudo_inverse_count; ++motor) {
    destination.pseudo_inverse[motor].r = source.pseudo_inverse[motor].r;
    destination.pseudo_inverse[motor].p = source.pseudo_inverse[motor].p;
    destination.pseudo_inverse[motor].y = source.pseudo_inverse[motor].y;
  }
  for (size_t element = 0U; element < 6U; ++element)
    destination.inertia[element] = source.inertia[element];
  return destination;
}

inline PMatrixConfiguration
toWire(const FlightControlPMatrixPseudoInverseWithInertia &source) {
  PMatrixConfiguration destination{};
  destination.motor_count = static_cast<uint8_t>(std::min(
      source.pseudo_inverse_count, static_cast<size_t>(kFlightMotorCount)));
  for (size_t motor = 0U; motor < destination.motor_count; ++motor) {
    destination.pseudo_inverse[motor].r = source.pseudo_inverse[motor].r;
    destination.pseudo_inverse[motor].p = source.pseudo_inverse[motor].p;
    destination.pseudo_inverse[motor].y = source.pseudo_inverse[motor].y;
  }
  for (size_t element = 0U; element < 6U; ++element)
    destination.inertia[element] = source.inertia[element];
  return destination;
}

inline FlightControlTorqueAllocationMatrixInv
toInternal(const TorqueAllocationConfiguration &source) {
  FlightControlTorqueAllocationMatrixInv destination{};
  destination.rows_count =
      std::min(static_cast<size_t>(source.row_count),
               static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t row = 0U; row < destination.rows_count; ++row) {
    destination.rows[row].x = source.rows[row].x;
    destination.rows[row].y = source.rows[row].y;
    destination.rows[row].z = source.rows[row].z;
  }
  return destination;
}

inline TorqueAllocationConfiguration
toWire(const FlightControlTorqueAllocationMatrixInv &source) {
  TorqueAllocationConfiguration destination{};
  destination.row_count = static_cast<uint8_t>(
      std::min(source.rows_count, static_cast<size_t>(kFlightMotorCount)));
  for (size_t row = 0U; row < destination.row_count; ++row) {
    destination.rows[row].x = source.rows[row].x;
    destination.rows[row].y = source.rows[row].y;
    destination.rows[row].z = source.rows[row].z;
  }
  return destination;
}

inline FlightControlDesireCoord
toInternal(const OffsetRotationConfiguration &source) {
  FlightControlDesireCoord destination{};
  destination.roll = source.roll;
  destination.pitch = source.pitch;
  destination.yaw = source.yaw;
  return destination;
}

inline OffsetRotationConfiguration
toWire(const FlightControlDesireCoord &source) {
  OffsetRotationConfiguration destination{};
  destination.roll = source.roll;
  destination.pitch = source.pitch;
  destination.yaw = source.yaw;
  return destination;
}

inline PositionControlConfig
toInternal(const PositionControlConfiguration &source) {
  PositionControlConfig destination{};
  for (size_t axis = 0U; axis < 3U; ++axis) {
    destination.position_p[axis] = source.position_p[axis];
    destination.position_i[axis] = source.position_i[axis];
    destination.velocity_d[axis] = source.velocity_d[axis];
    destination.limit_sum[axis] = source.limit_sum[axis];
    destination.limit_p[axis] = source.limit_p[axis];
    destination.limit_i[axis] = source.limit_i[axis];
    destination.limit_d[axis] = source.limit_d[axis];
    destination.limit_err_p[axis] = source.limit_err_p[axis];
    destination.limit_err_d[axis] = source.limit_err_d[axis];
    destination.integral_limit[axis] = source.integral_limit[axis];
  }
  destination.yaw_p = source.yaw_p;
  destination.yaw_i = source.yaw_i;
  destination.yaw_limit_sum = source.yaw_limit_sum;
  destination.yaw_limit_err_p = source.yaw_limit_err_p;
  destination.yaw_limit_err_i = source.yaw_limit_err_i;
  destination.yaw_limit_err_d = source.yaw_limit_err_d;
  destination.max_horizontal_acceleration = source.max_horizontal_acceleration;
  destination.max_vertical_acceleration = source.max_vertical_acceleration;
  destination.max_tilt_angle = source.max_tilt_angle;
  destination.motor_count =
      std::min(static_cast<size_t>(source.motor_count),
               static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0U; motor < destination.motor_count; ++motor) {
    destination.vertical_acceleration_to_thrust[motor] =
        source.vertical_acceleration_to_thrust[motor];
    destination.yaw_acceleration_to_thrust[motor] =
        source.yaw_acceleration_to_thrust[motor];
    destination.z_p_gain[motor] = source.z_p_gain[motor];
    destination.z_i_gain[motor] = source.z_i_gain[motor];
    destination.z_d_gain[motor] = source.z_d_gain[motor];
    destination.yaw_p_gain[motor] = source.yaw_p_gain[motor];
    destination.yaw_i_gain[motor] = source.yaw_i_gain[motor];
    destination.yaw_d_gain[motor] = source.yaw_d_gain[motor];
  }
  destination.use_lqi_gains = source.use_lqi_gains != 0U;
  destination.yaw_rate_feedback_on_spinal =
      source.yaw_rate_feedback_on_spinal != 0U;
  destination.start_roll_pitch_integration_height =
      source.start_roll_pitch_integration_height;
  destination.landing_err_z = source.landing_err_z;
  destination.safe_landing_height = source.safe_landing_height;
  destination.setpoint_timeout_ms = source.setpoint_timeout_ms;
  destination.rc_max_horizontal_velocity = source.rc_max_horizontal_velocity;
  destination.rc_max_vertical_velocity = source.rc_max_vertical_velocity;
  destination.rc_max_yaw_rate = source.rc_max_yaw_rate;
  destination.rc_deadzone = source.rc_deadzone;
  destination.rc_timeout_ms = source.rc_timeout_ms;
  destination.supervisor.takeoff_height = source.supervisor.takeoff_height;
  destination.supervisor.takeoff_position_tolerance =
      source.supervisor.takeoff_position_tolerance;
  destination.supervisor.takeoff_velocity_tolerance =
      source.supervisor.takeoff_velocity_tolerance;
  destination.supervisor.takeoff_stable_time_ms =
      source.supervisor.takeoff_stable_time_ms;
  destination.supervisor.landing_speed = source.supervisor.landing_speed;
  destination.supervisor.landed_height = source.supervisor.landed_height;
  destination.supervisor.landed_velocity = source.supervisor.landed_velocity;
  destination.supervisor.landed_stable_time_ms =
      source.supervisor.landed_stable_time_ms;
  destination.supervisor.rc_authority_timeout_ms =
      source.supervisor.rc_authority_timeout_ms;
  return destination;
}

inline PositionControlConfiguration
toWire(const PositionControlConfig &source) {
  PositionControlConfiguration destination{};
  for (size_t axis = 0U; axis < 3U; ++axis) {
    destination.position_p[axis] = source.position_p[axis];
    destination.position_i[axis] = source.position_i[axis];
    destination.velocity_d[axis] = source.velocity_d[axis];
    destination.limit_sum[axis] = source.limit_sum[axis];
    destination.limit_p[axis] = source.limit_p[axis];
    destination.limit_i[axis] = source.limit_i[axis];
    destination.limit_d[axis] = source.limit_d[axis];
    destination.limit_err_p[axis] = source.limit_err_p[axis];
    destination.limit_err_d[axis] = source.limit_err_d[axis];
    destination.integral_limit[axis] = source.integral_limit[axis];
  }
  destination.yaw_p = source.yaw_p;
  destination.yaw_i = source.yaw_i;
  destination.yaw_limit_sum = source.yaw_limit_sum;
  destination.yaw_limit_err_p = source.yaw_limit_err_p;
  destination.yaw_limit_err_i = source.yaw_limit_err_i;
  destination.yaw_limit_err_d = source.yaw_limit_err_d;
  destination.max_horizontal_acceleration = source.max_horizontal_acceleration;
  destination.max_vertical_acceleration = source.max_vertical_acceleration;
  destination.max_tilt_angle = source.max_tilt_angle;
  destination.motor_count = static_cast<uint8_t>(
      std::min(source.motor_count, static_cast<size_t>(kFlightMotorCount)));
  for (size_t motor = 0U; motor < destination.motor_count; ++motor) {
    destination.vertical_acceleration_to_thrust[motor] =
        source.vertical_acceleration_to_thrust[motor];
    destination.yaw_acceleration_to_thrust[motor] =
        source.yaw_acceleration_to_thrust[motor];
    destination.z_p_gain[motor] = source.z_p_gain[motor];
    destination.z_i_gain[motor] = source.z_i_gain[motor];
    destination.z_d_gain[motor] = source.z_d_gain[motor];
    destination.yaw_p_gain[motor] = source.yaw_p_gain[motor];
    destination.yaw_i_gain[motor] = source.yaw_i_gain[motor];
    destination.yaw_d_gain[motor] = source.yaw_d_gain[motor];
  }
  destination.use_lqi_gains = source.use_lqi_gains ? 1U : 0U;
  destination.yaw_rate_feedback_on_spinal =
      source.yaw_rate_feedback_on_spinal ? 1U : 0U;
  destination.start_roll_pitch_integration_height =
      source.start_roll_pitch_integration_height;
  destination.landing_err_z = source.landing_err_z;
  destination.safe_landing_height = source.safe_landing_height;
  destination.setpoint_timeout_ms = source.setpoint_timeout_ms;
  destination.rc_max_horizontal_velocity = source.rc_max_horizontal_velocity;
  destination.rc_max_vertical_velocity = source.rc_max_vertical_velocity;
  destination.rc_max_yaw_rate = source.rc_max_yaw_rate;
  destination.rc_deadzone = source.rc_deadzone;
  destination.rc_timeout_ms = source.rc_timeout_ms;
  destination.supervisor.takeoff_height = source.supervisor.takeoff_height;
  destination.supervisor.takeoff_position_tolerance =
      source.supervisor.takeoff_position_tolerance;
  destination.supervisor.takeoff_velocity_tolerance =
      source.supervisor.takeoff_velocity_tolerance;
  destination.supervisor.takeoff_stable_time_ms =
      source.supervisor.takeoff_stable_time_ms;
  destination.supervisor.landing_speed = source.supervisor.landing_speed;
  destination.supervisor.landed_height = source.supervisor.landed_height;
  destination.supervisor.landed_velocity = source.supervisor.landed_velocity;
  destination.supervisor.landed_stable_time_ms =
      source.supervisor.landed_stable_time_ms;
  destination.supervisor.rc_authority_timeout_ms =
      source.supervisor.rc_authority_timeout_ms;
  return destination;
}

inline HealthManagerConfig toInternal(const HealthConfig &source) {
  HealthManagerConfig destination{};
  destination.power.battery_cell_count = source.battery_cell_count;
  destination.power.low_percentage = source.battery_low_percentage;
  destination.power.hysteresis_percentage =
      source.battery_hysteresis_percentage;
  destination.power.high_cell_threshold = source.battery_high_cell_threshold;
  destination.power.resistance = source.battery_resistance;
  destination.power.resistance_voltage_rate =
      source.battery_resistance_voltage_rate;
  destination.power.hovering_current = source.battery_hovering_current;
  destination.power.debounce_ms = source.battery_debounce_ms;
  destination.sensor.primary_imu_timeout_ms = source.primary_imu_timeout_ms;
  destination.compute.control_loop_deadline_ms =
      source.control_loop_deadline_ms;
  destination.compute.miss_limit = source.control_loop_miss_limit;
  return destination;
}

inline HealthConfig toWire(const HealthManagerConfig &source) {
  HealthConfig destination{};
  destination.battery_cell_count = source.power.battery_cell_count;
  destination.battery_low_percentage = source.power.low_percentage;
  destination.battery_hysteresis_percentage =
      source.power.hysteresis_percentage;
  destination.battery_high_cell_threshold = source.power.high_cell_threshold;
  destination.battery_resistance = source.power.resistance;
  destination.battery_resistance_voltage_rate =
      source.power.resistance_voltage_rate;
  destination.battery_hovering_current = source.power.hovering_current;
  destination.battery_debounce_ms = source.power.debounce_ms;
  destination.primary_imu_timeout_ms = source.sensor.primary_imu_timeout_ms;
  destination.control_loop_deadline_ms =
      source.compute.control_loop_deadline_ms;
  destination.control_loop_miss_limit = source.compute.miss_limit;
  return destination;
}

inline FlightParameterSnapshot toWire(const FlightParameterPayload &source) {
  FlightParameterSnapshot destination{};
  destination.valid_fields = source.valid_fields;
  destination.motor_count = source.motor_count;
  destination.uav_model = source.uav_model;
  destination.gimbal_dof = source.gimbal_dof;
  destination.position_control_enabled = source.position_control_enabled;
  destination.attitude_control_enabled = source.attitude_control_enabled;
  destination.pwm = toWire(source.pwm);
  destination.attitude_gains = toWire(source.attitude_gains);
  destination.p_matrix = toWire(source.p_matrix);
  destination.torque_allocation = toWire(source.torque_allocation);
  destination.offset_rotation = toWire(source.offset_rotation);
  destination.position_control = toWire(source.position_control);
  destination.health = toWire(source.health);
  return destination;
}

inline FlightParameterPayload
toInternal(const FlightParameterSnapshot &source) {
  FlightParameterPayload destination{};
  destination.valid_fields = source.valid_fields;
  destination.motor_count = source.motor_count;
  destination.uav_model = source.uav_model;
  destination.gimbal_dof = source.gimbal_dof;
  destination.position_control_enabled = source.position_control_enabled;
  destination.attitude_control_enabled = source.attitude_control_enabled;
  destination.pwm = toInternal(source.pwm);
  destination.attitude_gains = toInternal(source.attitude_gains);
  destination.p_matrix = toInternal(source.p_matrix);
  destination.torque_allocation = toInternal(source.torque_allocation);
  destination.offset_rotation = toInternal(source.offset_rotation);
  destination.position_control = toInternal(source.position_control);
  destination.health = toInternal(source.health);
  return destination;
}

} // namespace conversion
} // namespace vehicle
} // namespace aerial
