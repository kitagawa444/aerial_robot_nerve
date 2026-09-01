#ifdef SIMULATION

#include "flight_control/simulation/flight_control_ros_module.h"

#include <algorithm>
#include <functional>

void FlightControlRosModule::init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                  StateEstimate *estimator, ThrusterManager *thruster)
{
  node_ = node;
  thruster_ = thruster;
  flight_control_.init(estimator, thruster, nullptr);

  if (!node_->has_parameter("rc_serial_port"))
  {
    (void)node_->declare_parameter<std::string>("rc_serial_port", "");
  }
  if (!node_->has_parameter("rc_serial_baud"))
  {
    (void)node_->declare_parameter<int64_t>("rc_serial_baud", crsf::DEFAULT_BAUD_RATE);
  }
  const std::string rc_serial_port = node_->get_parameter("rc_serial_port").as_string();
  const int64_t rc_serial_baud = node_->get_parameter("rc_serial_baud").as_int();
  crsf_ros_module_.init(node_, rc_serial_port,
                        rc_serial_baud > 0 ? static_cast<uint32_t>(rc_serial_baud) : crsf::DEFAULT_BAUD_RATE,
                        &flight_control_);

  if (!initialized_)
  {
    configureRosIo_();
    initialized_ = true;
  }
}

void FlightControlRosModule::update()
{
  crsf_ros_module_.update();
  flight_control_.update();
}

void FlightControlRosModule::activate()
{
  if (config_ack_pub_) config_ack_pub_->on_activate();
  if (control_term_pub_) control_term_pub_->on_activate();
  if (control_feedback_state_pub_) control_feedback_state_pub_->on_activate();
  if (gyro_moment_pub_) gyro_moment_pub_->on_activate();
  if (gimbal_control_pub_) gimbal_control_pub_->on_activate();
}

void FlightControlRosModule::deactivate()
{
  if (config_ack_pub_) config_ack_pub_->on_deactivate();
  if (control_term_pub_) control_term_pub_->on_deactivate();
  if (control_feedback_state_pub_) control_feedback_state_pub_->on_deactivate();
  if (gyro_moment_pub_) gyro_moment_pub_->on_deactivate();
  if (gimbal_control_pub_) gimbal_control_pub_->on_deactivate();
}

void FlightControlRosModule::publish()
{
  if (!node_) return;

  uint8_t ack = 0;
  if (flight_control_.consumeConfigAck(ack) && config_ack_pub_)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = ack;
    config_ack_pub_->publish(msg);
  }

  AttitudeController &att = flight_control_.getAttitudeController();
  if (att.getUavModel() == FlightControlUavModel::DRAGON)
  {
    if (att.controlFeedbackStatePublishReady(true) && control_feedback_state_pub_)
    {
      const FlightControlRpyTerm &src = att.getControlFeedbackState();
      spinal_msgs::msg::RollPitchYawTerm msg;
      msg.roll_p = src.roll_p;
      msg.roll_i = src.roll_i;
      msg.roll_d = src.roll_d;
      msg.pitch_p = src.pitch_p;
      msg.pitch_i = src.pitch_i;
      msg.pitch_d = src.pitch_d;
      msg.yaw_d = src.yaw_d;
      control_feedback_state_pub_->publish(msg);
    }
  }
  else if (att.controlTermPublishReady(true) && control_term_pub_)
  {
    const FlightControlRpyTerms &src = att.getControlTerms();
    spinal_msgs::msg::RollPitchYawTerms msg;
    const size_t n = std::min(src.motors_count, static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
    msg.motors.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      msg.motors[i].roll_p = src.motors[i].roll_p;
      msg.motors[i].roll_i = src.motors[i].roll_i;
      msg.motors[i].roll_d = src.motors[i].roll_d;
      msg.motors[i].pitch_p = src.motors[i].pitch_p;
      msg.motors[i].pitch_i = src.motors[i].pitch_i;
      msg.motors[i].pitch_d = src.motors[i].pitch_d;
      msg.motors[i].yaw_d = src.motors[i].yaw_d;
    }
    control_term_pub_->publish(msg);
  }

  if (gyro_moment_pub_)
  {
    std_msgs::msg::Float32MultiArray msg;
    const size_t n = std::min(static_cast<size_t>(att.getMotorNumber()),
                              static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
    const float *gyro = att.getGyroMomentCompensation();
    msg.data.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      msg.data[i] = gyro[i];
    }
    gyro_moment_pub_->publish(msg);
  }

  const size_t gimbal_count = std::min(att.getTargetGimbalAngleCount(),
                                       static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  if (gimbal_count > 0 && gimbal_control_pub_)
  {
    const float *angles = att.getTargetGimbalAngles();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.position.resize(gimbal_count);
    for (size_t i = 0; i < gimbal_count; ++i)
    {
      msg.position[i] = static_cast<double>(angles[i]);
    }
    gimbal_control_pub_->publish(msg);
  }
}

void FlightControlRosModule::configureRosIo_()
{
  if (!node_) return;

  flight_config_sub_ = node_->create_subscription<spinal_msgs::msg::FlightConfigCmd>(
      "flight_config_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::flightConfigCallback_, this, std::placeholders::_1));

  uav_info_sub_ = node_->create_subscription<spinal_msgs::msg::UavInfo>(
      "uav_info", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::uavInfoCallback_, this, std::placeholders::_1));

  gimbal_dof_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "gimbal_dof", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::gimbalDofCallback_, this, std::placeholders::_1));

  four_axis_cmd_sub_ = node_->create_subscription<spinal_msgs::msg::FourAxisCommand>(
      "four_axes/command", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::fourAxisCommandCallback_, this, std::placeholders::_1));

  rpy_gain_sub_ = node_->create_subscription<spinal_msgs::msg::RollPitchYawTerms>(
      "rpy/gain", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::rpyGainCallback_, this, std::placeholders::_1));

  p_matrix_sub_ = node_->create_subscription<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>(
      "p_matrix_pseudo_inverse_inertia", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::pMatrixCallback_, this, std::placeholders::_1));

  torque_allocation_sub_ = node_->create_subscription<spinal_msgs::msg::TorqueAllocationMatrixInv>(
      "torque_allocation_matrix_inv", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::torqueAllocationCallback_, this, std::placeholders::_1));

  offset_rot_sub_ = node_->create_subscription<spinal_msgs::msg::DesireCoord>(
      "desire_coordinate", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::offsetRotCallback_, this, std::placeholders::_1));

  sim_voltage_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "set_sim_voltage", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::simVoltageCallback_, this, std::placeholders::_1));

  position_config_sub_ = node_->create_subscription<spinal_msgs::msg::PositionControlConfig>(
      "position_control/config", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::positionConfigCallback_, this, std::placeholders::_1));

  position_setpoint_sub_ = node_->create_subscription<spinal_msgs::msg::PositionControlSetpoint>(
      "position_control/setpoint", rclcpp::SensorDataQoS(),
      std::bind(&FlightControlRosModule::positionSetpointCallback_, this, std::placeholders::_1));

  config_ack_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("flight_config_ack", rclcpp::QoS(1));
  control_term_pub_ = node_->create_publisher<spinal_msgs::msg::RollPitchYawTerms>("rpy/pid", rclcpp::QoS(1));
  control_feedback_state_pub_ = node_->create_publisher<spinal_msgs::msg::RollPitchYawTerm>("rpy/feedback_state",
                                                                                            rclcpp::QoS(1));
  gyro_moment_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("gyro_moment_compensation",
                                                                               rclcpp::QoS(1));
  gimbal_control_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("gimbals_ctrl", rclcpp::QoS(1));

  att_control_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "set_attitude_control",
      std::bind(&FlightControlRosModule::attitudeControlCallback_, this, std::placeholders::_1, std::placeholders::_2));

  position_control_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "set_position_control",
      std::bind(&FlightControlRosModule::positionControlCallback_, this, std::placeholders::_1, std::placeholders::_2));
}

void FlightControlRosModule::flightConfigCallback_(const spinal_msgs::msg::FlightConfigCmd::SharedPtr msg)
{
  if (!msg) return;
  if (!flight_control_.applyFlightConfig(msg->cmd) && msg->cmd == FlightControlCommand::ARM_ON_CMD)
  {
    const PositionController &position = flight_control_.getPositionController();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[sim] ARM rejected: attitude_ready=%d position_enabled=%d "
                         "position_configured=%d "
                         "position_state_valid=%d position_ready=%d active=%d rpy_gains=%d "
                         "allocation=%d",
                         flight_control_.getAttitudeController().activated(), position.enabled(), position.configured(),
                         flight_control_.positionControlStateValid(), flight_control_.positionControlReady(),
                         position.active(), flight_control_.getAttitudeController().rpyGainsConfigured(),
                         flight_control_.getAttitudeController().torqueAllocationConfigured());
  }
}

void FlightControlRosModule::uavInfoCallback_(const spinal_msgs::msg::UavInfo::SharedPtr msg)
{
  if (!msg) return;
  flight_control_.applyUavInfo(msg->motor_num, static_cast<int8_t>(msg->uav_model));
}

void FlightControlRosModule::gimbalDofCallback_(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) return;
  flight_control_.applyGimbalDof(msg->data);
}

void FlightControlRosModule::fourAxisCommandCallback_(const spinal_msgs::msg::FourAxisCommand::SharedPtr msg)
{
  if (!msg) return;

  FlightControlFourAxisCommand cmd;
  cmd.angles[0] = msg->angles[0];
  cmd.angles[1] = msg->angles[1];
  cmd.angles[2] = msg->angles[2];
  cmd.base_thrust_count = std::min(msg->base_thrust.size(), static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t i = 0; i < cmd.base_thrust_count; ++i)
  {
    cmd.base_thrust[i] = msg->base_thrust[i];
  }
  (void)flight_control_.applyFourAxisCommand(cmd);
}

void FlightControlRosModule::rpyGainCallback_(const spinal_msgs::msg::RollPitchYawTerms::SharedPtr msg)
{
  if (!msg) return;

  FlightControlRpyTerms gains;
  gains.motors_count = std::min(msg->motors.size(), static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t i = 0; i < gains.motors_count; ++i)
  {
    gains.motors[i].roll_p = msg->motors[i].roll_p;
    gains.motors[i].roll_i = msg->motors[i].roll_i;
    gains.motors[i].roll_d = msg->motors[i].roll_d;
    gains.motors[i].pitch_p = msg->motors[i].pitch_p;
    gains.motors[i].pitch_i = msg->motors[i].pitch_i;
    gains.motors[i].pitch_d = msg->motors[i].pitch_d;
    gains.motors[i].yaw_d = msg->motors[i].yaw_d;
  }
  if (!flight_control_.applyRpyGains(gains))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected RPY gains for %zu motors", gains.motors_count);
  }
}

void FlightControlRosModule::pMatrixCallback_(const spinal_msgs::msg::PMatrixPseudoInverseWithInertia::SharedPtr msg)
{
  if (!msg) return;

  FlightControlPMatrixPseudoInverseWithInertia dst;
  dst.pseudo_inverse_count = std::min(msg->pseudo_inverse.size(), static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t i = 0; i < dst.pseudo_inverse_count; ++i)
  {
    dst.pseudo_inverse[i].r = msg->pseudo_inverse[i].r;
    dst.pseudo_inverse[i].p = msg->pseudo_inverse[i].p;
    dst.pseudo_inverse[i].y = msg->pseudo_inverse[i].y;
  }
  for (size_t i = 0; i < 6; ++i)
  {
    dst.inertia[i] = msg->inertia[i];
  }
  (void)flight_control_.applyPMatrixInertia(dst);
}

void FlightControlRosModule::torqueAllocationCallback_(const spinal_msgs::msg::TorqueAllocationMatrixInv::SharedPtr msg)
{
  if (!msg) return;

  FlightControlTorqueAllocationMatrixInv dst;
  dst.rows_count = std::min(msg->rows.size(), static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t i = 0; i < dst.rows_count; ++i)
  {
    dst.rows[i].x = msg->rows[i].x;
    dst.rows[i].y = msg->rows[i].y;
    dst.rows[i].z = msg->rows[i].z;
  }
  if (!flight_control_.applyTorqueAllocationMatrixInv(dst))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected torque allocation for %zu motors", dst.rows_count);
  }
}

void FlightControlRosModule::offsetRotCallback_(const spinal_msgs::msg::DesireCoord::SharedPtr msg)
{
  if (!msg) return;

  FlightControlDesireCoord dst;
  dst.roll = msg->roll;
  dst.pitch = msg->pitch;
  dst.yaw = msg->yaw;
  flight_control_.applyOffsetRotation(dst);
}

void FlightControlRosModule::simVoltageCallback_(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!msg || thruster_ == nullptr) return;
  thruster_->setSimVoltage(msg->data);
}

void FlightControlRosModule::positionConfigCallback_(const spinal_msgs::msg::PositionControlConfig::SharedPtr msg)
{
  if (!msg) return;
  const size_t motor_count = msg->vertical_acceleration_to_thrust.size();
  if (msg->use_lqi_gains && (msg->yaw_acceleration_to_thrust.size() != motor_count ||
                             msg->z_p_gain.size() != motor_count || msg->z_i_gain.size() != motor_count ||
                             msg->z_d_gain.size() != motor_count || msg->yaw_p_gain.size() != motor_count ||
                             msg->yaw_i_gain.size() != motor_count || msg->yaw_d_gain.size() != motor_count))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected incomplete LQI position-control configuration");
    return;
  }
  PositionControlConfig config;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    config.position_p[axis] = msg->position_p[axis];
    config.position_i[axis] = msg->position_i[axis];
    config.velocity_d[axis] = msg->velocity_d[axis];
    config.limit_sum[axis] = msg->limit_sum[axis];
    config.limit_p[axis] = msg->limit_p[axis];
    config.limit_i[axis] = msg->limit_i[axis];
    config.limit_d[axis] = msg->limit_d[axis];
    config.limit_err_p[axis] = msg->limit_err_p[axis];
    config.limit_err_d[axis] = msg->limit_err_d[axis];
    config.integral_limit[axis] = msg->integral_limit[axis];
  }
  config.yaw_p = msg->yaw_p;
  config.yaw_i = msg->yaw_i;
  config.yaw_limit_sum = msg->yaw_limit_sum;
  config.yaw_limit_err_p = msg->yaw_limit_err_p;
  config.yaw_limit_err_i = msg->yaw_limit_err_i;
  config.yaw_limit_err_d = msg->yaw_limit_err_d;
  config.max_horizontal_acceleration = msg->max_horizontal_acceleration;
  config.max_vertical_acceleration = msg->max_vertical_acceleration;
  config.max_tilt_angle = msg->max_tilt_angle;
  config.motor_count = std::min(motor_count, static_cast<size_t>(MAX_FLIGHT_CONTROL_MOTOR_NUM));
  for (size_t motor = 0; motor < config.motor_count; ++motor)
  {
    config.vertical_acceleration_to_thrust[motor] = msg->vertical_acceleration_to_thrust[motor];
    if (motor < msg->yaw_acceleration_to_thrust.size())
      config.yaw_acceleration_to_thrust[motor] = msg->yaw_acceleration_to_thrust[motor];
    if (motor < msg->z_p_gain.size()) config.z_p_gain[motor] = msg->z_p_gain[motor];
    if (motor < msg->z_i_gain.size()) config.z_i_gain[motor] = msg->z_i_gain[motor];
    if (motor < msg->z_d_gain.size()) config.z_d_gain[motor] = msg->z_d_gain[motor];
    if (motor < msg->yaw_p_gain.size()) config.yaw_p_gain[motor] = msg->yaw_p_gain[motor];
    if (motor < msg->yaw_i_gain.size()) config.yaw_i_gain[motor] = msg->yaw_i_gain[motor];
    if (motor < msg->yaw_d_gain.size()) config.yaw_d_gain[motor] = msg->yaw_d_gain[motor];
  }
  config.use_lqi_gains = msg->use_lqi_gains;
  config.yaw_rate_feedback_on_spinal = msg->yaw_rate_feedback_on_spinal;
  config.start_roll_pitch_integration_height = msg->start_roll_pitch_integration_height;
  config.landing_err_z = msg->landing_err_z;
  config.safe_landing_height = msg->safe_landing_height;
  config.setpoint_timeout_ms = msg->setpoint_timeout_ms;
  config.rc_max_horizontal_velocity = msg->rc_max_horizontal_velocity;
  config.rc_max_vertical_velocity = msg->rc_max_vertical_velocity;
  config.rc_max_yaw_rate = msg->rc_max_yaw_rate;
  config.rc_deadzone = msg->rc_deadzone;
  config.rc_timeout_ms = msg->rc_timeout_ms;
  (void)flight_control_.applyPositionControlConfig(config);
}

void FlightControlRosModule::positionSetpointCallback_(const spinal_msgs::msg::PositionControlSetpoint::SharedPtr msg)
{
  if (!msg) return;
  PositionControlSetpoint setpoint;
  setpoint.position = ap::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
  setpoint.velocity = ap::Vector3f(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  setpoint.acceleration = ap::Vector3f(msg->acceleration[0], msg->acceleration[1], msg->acceleration[2]);
  setpoint.yaw = msg->yaw;
  setpoint.yaw_rate = msg->yaw_rate;
  setpoint.yaw_acceleration = msg->yaw_acceleration;
  setpoint.initial_height = msg->initial_height;
  setpoint.horizontal_control_mode = msg->horizontal_control_mode;
  setpoint.active = msg->active;
  setpoint.manual_control_allowed = msg->manual_control_allowed;
  setpoint.landing = msg->landing;
  (void)flight_control_.applyPositionControlSetpoint(setpoint);
}

void FlightControlRosModule::attitudeControlCallback_(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (!req || !res) return;
  flight_control_.setAttitudeControlFlag(req->data);
  res->success = true;
}

void FlightControlRosModule::positionControlCallback_(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (!req || !res) return;
  flight_control_.setPositionControlEnabled(req->data);
  res->success = flight_control_.getPositionController().enabled() == req->data;
  res->message = res->success ? "position control mode updated" : "disarm before changing position control mode";
}

#endif  // SIMULATION
