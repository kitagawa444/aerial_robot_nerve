#ifdef SIMULATION

#include "flight_control/simulation/flight_control_ros_module.h"

#include "flight_control/flight_control_ros_adapter.h"

#include <algorithm>
#include <functional>

void FlightControlRosModule::init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                  StateEstimate *estimator, ThrusterManager *thruster)
{
  node_ = node;
  last_flight_status_publish_time_ = node_->now();
  thruster_ = thruster;
  flight_control_.init(estimator, thruster, nullptr);
  flight_control_.setRosLinkState(FlightLinkState::CONNECTING);

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
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_.setRosLinkState(FlightLinkState::CONNECTED);
  crsf_ros_module_.update();
  flight_control_.update();
}

void FlightControlRosModule::activate()
{
  if (config_ack_pub_) config_ack_pub_->on_activate();
  if (flight_status_pub_) flight_status_pub_->on_activate();
  if (control_term_pub_) control_term_pub_->on_activate();
  if (control_feedback_state_pub_) control_feedback_state_pub_->on_activate();
  if (gyro_moment_pub_) gyro_moment_pub_->on_activate();
  if (gimbal_control_pub_) gimbal_control_pub_->on_activate();
}

void FlightControlRosModule::deactivate()
{
  {
    const std::lock_guard<std::mutex> lock(control_mutex_);
    flight_control_.setRosLinkState(FlightLinkState::DISCONNECTED);
  }
  if (config_ack_pub_) config_ack_pub_->on_deactivate();
  if (flight_status_pub_) flight_status_pub_->on_deactivate();
  if (control_term_pub_) control_term_pub_->on_deactivate();
  if (control_feedback_state_pub_) control_feedback_state_pub_->on_deactivate();
  if (gyro_moment_pub_) gyro_moment_pub_->on_deactivate();
  if (gimbal_control_pub_) gimbal_control_pub_->on_deactivate();
}

void FlightControlRosModule::publish()
{
  if (!node_) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);

  uint8_t ack = 0;
  if (flight_control_.consumeConfigAck(ack) && config_ack_pub_)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = ack;
    config_ack_pub_->publish(msg);
  }

  const FlightSupervisorStatus &status = flight_control_.getSupervisor().status();
  const rclcpp::Time now = node_->now();
  if (flight_status_pub_ &&
      (status.sequence != last_flight_status_sequence_ || (now - last_flight_status_publish_time_).seconds() >= 0.1))
  {
    spinal_msgs::msg::FlightStatus msg;
    msg.stamp = now;
    flight_control_ros::fillFlightStatus(msg, status);
    flight_status_pub_->publish(msg);
    last_flight_status_sequence_ = status.sequence;
    last_flight_status_publish_time_ = now;
  }

  AttitudeController &att = flight_control_.getAttitudeController();
  if (att.getUavModel() == FlightControlUavModel::DRAGON)
  {
    if (att.controlFeedbackStatePublishReady(true) && control_feedback_state_pub_)
    {
      const FlightControlRpyTerm &src = att.getControlFeedbackState();
      spinal_msgs::msg::RollPitchYawTerm msg;
      flight_control_ros::fillRollPitchYawTerm(msg, src);
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
      flight_control_ros::fillRollPitchYawTerm(msg.motors[i], src.motors[i]);
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

  health_config_sub_ = node_->create_subscription<spinal_msgs::msg::HealthConfig>(
      "health/config", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::healthConfigCallback_, this, std::placeholders::_1));

  position_setpoint_sub_ = node_->create_subscription<spinal_msgs::msg::PositionControlSetpoint>(
      "position_control/setpoint", rclcpp::SensorDataQoS(),
      std::bind(&FlightControlRosModule::positionSetpointCallback_, this, std::placeholders::_1));

  network_heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "network/heartbeat", rclcpp::SystemDefaultsQoS(),
      std::bind(&FlightControlRosModule::networkHeartbeatCallback_, this, std::placeholders::_1));

  config_ack_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("flight_config_ack", rclcpp::QoS(1));
  flight_status_pub_ = node_->create_publisher<spinal_msgs::msg::FlightStatus>("fc/flight_status", rclcpp::QoS(1));
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
  const std::lock_guard<std::mutex> lock(control_mutex_);
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
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_.applyUavInfo(msg->motor_num, static_cast<int8_t>(msg->uav_model));
}

void FlightControlRosModule::gimbalDofCallback_(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_.applyGimbalDof(msg->data);
}

void FlightControlRosModule::fourAxisCommandCallback_(const spinal_msgs::msg::FourAxisCommand::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  (void)flight_control_ros::applyFourAxisCommand(flight_control_, *msg);
}

void FlightControlRosModule::rpyGainCallback_(const spinal_msgs::msg::RollPitchYawTerms::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  if (!flight_control_ros::applyRpyGains(flight_control_, *msg))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected RPY gains for %zu motors", msg->motors.size());
  }
}

void FlightControlRosModule::pMatrixCallback_(const spinal_msgs::msg::PMatrixPseudoInverseWithInertia::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  (void)flight_control_ros::applyPMatrixInertia(flight_control_, *msg);
}

void FlightControlRosModule::torqueAllocationCallback_(const spinal_msgs::msg::TorqueAllocationMatrixInv::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  if (!flight_control_ros::applyTorqueAllocationMatrixInv(flight_control_, *msg))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected torque allocation for %zu motors", msg->rows.size());
  }
}

void FlightControlRosModule::offsetRotCallback_(const spinal_msgs::msg::DesireCoord::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_ros::applyOffsetRotation(flight_control_, *msg);
}

void FlightControlRosModule::simVoltageCallback_(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!msg || thruster_ == nullptr) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  thruster_->setSimVoltage(msg->data);
}

void FlightControlRosModule::positionConfigCallback_(const spinal_msgs::msg::PositionControlConfig::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  if (!flight_control_ros::applyPositionControlConfig(flight_control_, *msg))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected position-control configuration");
  }
}

void FlightControlRosModule::healthConfigCallback_(const spinal_msgs::msg::HealthConfig::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  if (!flight_control_ros::applyHealthConfig(flight_control_, *msg))
  {
    RCLCPP_WARN(node_->get_logger(), "[sim] Rejected health configuration");
  }
}

void FlightControlRosModule::positionSetpointCallback_(const spinal_msgs::msg::PositionControlSetpoint::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  (void)flight_control_ros::applyPositionControlSetpoint(flight_control_, *msg);
}

void FlightControlRosModule::networkHeartbeatCallback_(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (!msg) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_.noteNetworkHeartbeat();
}

void FlightControlRosModule::attitudeControlCallback_(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (!req || !res) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_.setAttitudeControlFlag(req->data);
  res->success = true;
}

void FlightControlRosModule::positionControlCallback_(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (!req || !res) return;
  const std::lock_guard<std::mutex> lock(control_mutex_);
  flight_control_.setPositionControlEnabled(req->data);
  res->success = flight_control_.getPositionController().enabled() == req->data;
  res->message = res->success ? "position control mode updated" : "disarm before changing position control mode";
}

#endif  // SIMULATION
