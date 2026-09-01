#include "flight_control/flight_control_ros_adapter.h"

#include <algorithm>

#ifdef SIMULATION
#define FC_ROS_SEQUENCE_SIZE(sequence) ((sequence).size())
#define FC_ROS_SEQUENCE_AT(sequence, index) ((sequence)[index])
#else
#define FC_ROS_SEQUENCE_SIZE(sequence) ((sequence).size)
#define FC_ROS_SEQUENCE_AT(sequence, index) ((sequence).data[index])
#endif

namespace flight_control_ros
{
namespace
{
template <typename HealthStatusMsg> void fillHealthStatus(HealthStatusMsg &msg, const HealthComponentStatus &src)
{
  msg.state = src.state;
  msg.reason_mask = src.reason_mask;
  msg.last_change_ms = src.last_change_ms;
}

template <typename HealthReportMsg> void fillHealthReport(HealthReportMsg &msg, const FlightHealthReport &src)
{
  fillHealthStatus(msg.overall, src.overall);
  fillHealthStatus(msg.flight, src.flight);
  fillHealthStatus(msg.link.summary, src.link.summary);
  msg.link.ros = src.link.ros;
  msg.link.network = src.link.network;
  msg.link.rc = src.link.rc;
  fillHealthStatus(msg.power, src.power);
  fillHealthStatus(msg.sensor, src.sensor);
  fillHealthStatus(msg.estimation, src.estimation);
  fillHealthStatus(msg.actuator, src.actuator);
  fillHealthStatus(msg.compute, src.compute);
  fillHealthStatus(msg.config, src.config);
  msg.requested_action = src.requested_action;
  msg.action_source = src.action_source;
  msg.action_reason_mask = src.action_reason_mask;
  msg.action_since_ms = src.action_since_ms;
  msg.battery_voltage = src.battery_voltage;
  msg.battery_compensated_voltage = src.battery_compensated_voltage;
  msg.battery_percentage = src.battery_percentage;
  msg.estimation_validity = src.estimation_validity;
  msg.rejected_measurements = src.rejected_measurements;
}
}  // namespace

bool applyFourAxisCommand(FlightControl &flight_control, const FourAxisCommandMsg &msg)
{
  FlightControlFourAxisCommand command;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    command.angles[axis] = msg.angles[axis];
  }
  command.base_thrust_count = std::min(FC_ROS_SEQUENCE_SIZE(msg.base_thrust),
                                       static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0; motor < command.base_thrust_count; ++motor)
  {
    command.base_thrust[motor] = FC_ROS_SEQUENCE_AT(msg.base_thrust, motor);
  }
  return flight_control.applyFourAxisCommand(command);
}

bool applyRpyGains(FlightControl &flight_control, const RollPitchYawTermsMsg &msg)
{
  FlightControlRpyTerms gains;
  gains.motors_count = std::min(FC_ROS_SEQUENCE_SIZE(msg.motors), static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0; motor < gains.motors_count; ++motor)
  {
    const auto &src = FC_ROS_SEQUENCE_AT(msg.motors, motor);
    FlightControlRpyTerm &dst = gains.motors[motor];
    dst.roll_p = src.roll_p;
    dst.roll_i = src.roll_i;
    dst.roll_d = src.roll_d;
    dst.pitch_p = src.pitch_p;
    dst.pitch_i = src.pitch_i;
    dst.pitch_d = src.pitch_d;
    dst.yaw_d = src.yaw_d;
  }
  return flight_control.applyRpyGains(gains);
}

bool applyPMatrixInertia(FlightControl &flight_control, const PMatrixMsg &msg)
{
  FlightControlPMatrixPseudoInverseWithInertia matrix;
  matrix.pseudo_inverse_count = std::min(FC_ROS_SEQUENCE_SIZE(msg.pseudo_inverse),
                                         static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0; motor < matrix.pseudo_inverse_count; ++motor)
  {
    const auto &src = FC_ROS_SEQUENCE_AT(msg.pseudo_inverse, motor);
    matrix.pseudo_inverse[motor].r = src.r;
    matrix.pseudo_inverse[motor].p = src.p;
    matrix.pseudo_inverse[motor].y = src.y;
  }
  for (size_t element = 0; element < 6; ++element)
  {
    matrix.inertia[element] = msg.inertia[element];
  }
  return flight_control.applyPMatrixInertia(matrix);
}

bool applyTorqueAllocationMatrixInv(FlightControl &flight_control, const TorqueAllocationMsg &msg)
{
  FlightControlTorqueAllocationMatrixInv matrix;
  matrix.rows_count = std::min(FC_ROS_SEQUENCE_SIZE(msg.rows), static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0; motor < matrix.rows_count; ++motor)
  {
    const auto &src = FC_ROS_SEQUENCE_AT(msg.rows, motor);
    matrix.rows[motor].x = src.x;
    matrix.rows[motor].y = src.y;
    matrix.rows[motor].z = src.z;
  }
  return flight_control.applyTorqueAllocationMatrixInv(matrix);
}

void applyOffsetRotation(FlightControl &flight_control, const DesireCoordMsg &msg)
{
  FlightControlDesireCoord coordinate;
  coordinate.roll = msg.roll;
  coordinate.pitch = msg.pitch;
  coordinate.yaw = msg.yaw;
  flight_control.applyOffsetRotation(coordinate);
}

bool applyPositionControlConfig(FlightControl &flight_control, const PositionControlConfigMsg &msg)
{
  const size_t motor_count = FC_ROS_SEQUENCE_SIZE(msg.vertical_acceleration_to_thrust);
  if (msg.use_lqi_gains &&
      (FC_ROS_SEQUENCE_SIZE(msg.yaw_acceleration_to_thrust) != motor_count ||
       FC_ROS_SEQUENCE_SIZE(msg.z_p_gain) != motor_count || FC_ROS_SEQUENCE_SIZE(msg.z_i_gain) != motor_count ||
       FC_ROS_SEQUENCE_SIZE(msg.z_d_gain) != motor_count || FC_ROS_SEQUENCE_SIZE(msg.yaw_p_gain) != motor_count ||
       FC_ROS_SEQUENCE_SIZE(msg.yaw_i_gain) != motor_count || FC_ROS_SEQUENCE_SIZE(msg.yaw_d_gain) != motor_count))
  {
    return false;
  }

  PositionControlConfig config;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    config.position_p[axis] = msg.position_p[axis];
    config.position_i[axis] = msg.position_i[axis];
    config.velocity_d[axis] = msg.velocity_d[axis];
    config.limit_sum[axis] = msg.limit_sum[axis];
    config.limit_p[axis] = msg.limit_p[axis];
    config.limit_i[axis] = msg.limit_i[axis];
    config.limit_d[axis] = msg.limit_d[axis];
    config.limit_err_p[axis] = msg.limit_err_p[axis];
    config.limit_err_d[axis] = msg.limit_err_d[axis];
    config.integral_limit[axis] = msg.integral_limit[axis];
  }
  config.yaw_p = msg.yaw_p;
  config.yaw_i = msg.yaw_i;
  config.yaw_limit_sum = msg.yaw_limit_sum;
  config.yaw_limit_err_p = msg.yaw_limit_err_p;
  config.yaw_limit_err_i = msg.yaw_limit_err_i;
  config.yaw_limit_err_d = msg.yaw_limit_err_d;
  config.max_horizontal_acceleration = msg.max_horizontal_acceleration;
  config.max_vertical_acceleration = msg.max_vertical_acceleration;
  config.max_tilt_angle = msg.max_tilt_angle;
  config.motor_count = std::min(motor_count, static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0; motor < config.motor_count; ++motor)
  {
    config.vertical_acceleration_to_thrust[motor] = FC_ROS_SEQUENCE_AT(msg.vertical_acceleration_to_thrust, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.yaw_acceleration_to_thrust))
      config.yaw_acceleration_to_thrust[motor] = FC_ROS_SEQUENCE_AT(msg.yaw_acceleration_to_thrust, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.z_p_gain)) config.z_p_gain[motor] = FC_ROS_SEQUENCE_AT(msg.z_p_gain, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.z_i_gain)) config.z_i_gain[motor] = FC_ROS_SEQUENCE_AT(msg.z_i_gain, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.z_d_gain)) config.z_d_gain[motor] = FC_ROS_SEQUENCE_AT(msg.z_d_gain, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.yaw_p_gain))
      config.yaw_p_gain[motor] = FC_ROS_SEQUENCE_AT(msg.yaw_p_gain, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.yaw_i_gain))
      config.yaw_i_gain[motor] = FC_ROS_SEQUENCE_AT(msg.yaw_i_gain, motor);
    if (motor < FC_ROS_SEQUENCE_SIZE(msg.yaw_d_gain))
      config.yaw_d_gain[motor] = FC_ROS_SEQUENCE_AT(msg.yaw_d_gain, motor);
  }
  config.use_lqi_gains = msg.use_lqi_gains;
  config.yaw_rate_feedback_on_spinal = msg.yaw_rate_feedback_on_spinal;
  config.start_roll_pitch_integration_height = msg.start_roll_pitch_integration_height;
  config.landing_err_z = msg.landing_err_z;
  config.safe_landing_height = msg.safe_landing_height;
  config.setpoint_timeout_ms = msg.setpoint_timeout_ms;
  config.rc_max_horizontal_velocity = msg.rc_max_horizontal_velocity;
  config.rc_max_vertical_velocity = msg.rc_max_vertical_velocity;
  config.rc_max_yaw_rate = msg.rc_max_yaw_rate;
  config.rc_deadzone = msg.rc_deadzone;
  config.rc_timeout_ms = msg.rc_timeout_ms;
  config.supervisor.takeoff_height = msg.takeoff_height;
  config.supervisor.takeoff_position_tolerance = msg.takeoff_position_tolerance;
  config.supervisor.takeoff_velocity_tolerance = msg.takeoff_velocity_tolerance;
  config.supervisor.takeoff_stable_time_ms = msg.takeoff_stable_time_ms;
  config.supervisor.landing_speed = msg.landing_speed;
  config.supervisor.landed_height = msg.landed_height;
  config.supervisor.landed_velocity = msg.landed_velocity;
  config.supervisor.landed_stable_time_ms = msg.landed_stable_time_ms;
  config.supervisor.rc_authority_timeout_ms = msg.rc_authority_timeout_ms;
  return flight_control.applyPositionControlConfig(config);
}

bool applyHealthConfig(FlightControl &flight_control, const HealthConfigMsg &msg)
{
  HealthManagerConfig config;
  config.power.battery_cell_count = msg.battery_cell_count;
  config.power.low_percentage = msg.battery_low_percentage;
  config.power.hysteresis_percentage = msg.battery_hysteresis_percentage;
  config.power.high_cell_threshold = msg.battery_high_cell_threshold;
  config.power.resistance = msg.battery_resistance;
  config.power.resistance_voltage_rate = msg.battery_resistance_voltage_rate;
  config.power.hovering_current = msg.battery_hovering_current;
  config.power.debounce_ms = msg.battery_debounce_ms;
  config.sensor.primary_imu_timeout_ms = msg.primary_imu_timeout_ms;
  config.compute.control_loop_deadline_ms = msg.control_loop_deadline_ms;
  config.compute.miss_limit = msg.control_loop_miss_limit;
  return flight_control.applyHealthConfig(config);
}

bool applyPositionControlSetpoint(FlightControl &flight_control, const PositionControlSetpointMsg &msg)
{
  PositionControlSetpoint setpoint;
  setpoint.position = ap::Vector3f(msg.position[0], msg.position[1], msg.position[2]);
  setpoint.velocity = ap::Vector3f(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
  setpoint.acceleration = ap::Vector3f(msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]);
  setpoint.yaw = msg.yaw;
  setpoint.yaw_rate = msg.yaw_rate;
  setpoint.yaw_acceleration = msg.yaw_acceleration;
  setpoint.initial_height = msg.initial_height;
  setpoint.horizontal_control_mode = msg.horizontal_control_mode;
  setpoint.active = msg.active;
  setpoint.manual_control_allowed = msg.manual_control_allowed;
  setpoint.landing = msg.landing;
  return flight_control.applyPositionControlSetpoint(setpoint);
}

void fillRollPitchYawTerm(RollPitchYawTermMsg &msg, const FlightControlRpyTerm &src)
{
  msg.roll_p = src.roll_p;
  msg.roll_i = src.roll_i;
  msg.roll_d = src.roll_d;
  msg.pitch_p = src.pitch_p;
  msg.pitch_i = src.pitch_i;
  msg.pitch_d = src.pitch_d;
  msg.yaw_d = src.yaw_d;
}

void fillFlightStatus(FlightStatusMsg &msg, const FlightSupervisorStatus &src)
{
  msg.sequence = src.sequence;
  msg.command_sequence = src.command_sequence;
  msg.arming_state = src.arming_state;
  msg.flight_phase = src.flight_phase;
  msg.control_mode = src.control_mode;
  msg.user_intention = src.user_intention;
  msg.authority = src.authority;
  msg.failsafe = src.failsafe;
  msg.last_command = src.last_command;
  msg.last_command_source = src.last_command_source;
  msg.last_command_result = src.last_command_result;
  msg.transition_reason = src.transition_reason;
  msg.ready_to_arm = src.ready_to_arm;
  msg.attitude_ready = src.attitude_ready;
  msg.position_ready = src.position_ready;
  msg.rc_connected = src.rc_connected;
  msg.ros_link_state = src.ros_link_state;
  msg.network_link_state = src.network_link_state;
  msg.rc_link_state = src.rc_link_state;
  fillHealthReport(msg.health, src.health);
  msg.takeoff_reference_height = src.takeoff_reference_height;
  msg.takeoff_target_height = src.takeoff_target_height;
}

}  // namespace flight_control_ros

#undef FC_ROS_SEQUENCE_SIZE
#undef FC_ROS_SEQUENCE_AT
