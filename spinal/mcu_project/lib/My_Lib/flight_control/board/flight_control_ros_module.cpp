#ifndef SIMULATION

#include "flight_control/board/flight_control_ros_module.h"

FlightControlRosModule *FlightControlRosModule::instance_ = nullptr;

void FlightControlRosModule::init_hw(StateEstimate *estimator, ThrusterManager *thruster, DirectServo *servo,
                                     osMutexId *control_mutex)
{
  control_mutex_ = control_mutex;
  flight_control_.init(estimator, thruster, servo);
}

void FlightControlRosModule::create_entities(rcl_node_t &node)
{
  reserve_entities();
  instance_ = this;

  spinal_msgs__msg__FlightConfigCmd__init(&flight_config_msg_);
  spinal_msgs__msg__UavInfo__init(&uav_info_msg_);
  std_msgs__msg__UInt8__init(&gimbal_dof_msg_);
  spinal_msgs__msg__FourAxisCommand__init(&four_axis_cmd_msg_);
  spinal_msgs__msg__RollPitchYawTerms__init(&rpy_gain_msg_);
  spinal_msgs__msg__PMatrixPseudoInverseWithInertia__init(&p_matrix_msg_);
  spinal_msgs__msg__TorqueAllocationMatrixInv__init(&torque_allocation_msg_);
  spinal_msgs__msg__DesireCoord__init(&offset_rot_msg_);
  spinal_msgs__msg__PositionControlConfig__init(&position_config_msg_);
  spinal_msgs__msg__PositionControlSetpoint__init(&position_setpoint_msg_);
  std_msgs__msg__UInt8__init(&config_ack_msg_);
  spinal_msgs__msg__RollPitchYawTerms__init(&control_term_msg_);
  spinal_msgs__msg__RollPitchYawTerm__init(&control_feedback_state_msg_);
  std_srvs__srv__SetBool_Request__init(&att_control_req_);
  std_srvs__srv__SetBool_Response__init(&att_control_res_);
  std_srvs__srv__SetBool_Request__init(&position_control_req_);
  std_srvs__srv__SetBool_Response__init(&position_control_res_);

  configure_message_storage_();

  (void)init_subscription_default(
      node, flight_config_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, FlightConfigCmd), "flight_config_cmd",
      &flight_config_msg_, &FlightControlRosModule::flightConfigCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, position_config_sub_,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PositionControlConfig),
                                  "position_control/config", &position_config_msg_,
                                  &FlightControlRosModule::positionConfigCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, position_setpoint_sub_,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PositionControlSetpoint),
                                  "position_control/setpoint", &position_setpoint_msg_,
                                  &FlightControlRosModule::positionSetpointCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, uav_info_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, UavInfo),
                                  "uav_info", &uav_info_msg_, &FlightControlRosModule::uavInfoCallbackStatic_,
                                  ON_NEW_DATA);

  (void)init_subscription_default(node, gimbal_dof_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                                  "gimbal_dof", &gimbal_dof_msg_, &FlightControlRosModule::gimbalDofCallbackStatic_,
                                  ON_NEW_DATA);

  (void)init_subscription_default(
      node, four_axis_cmd_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, FourAxisCommand), "four_axes/command",
      &four_axis_cmd_msg_, &FlightControlRosModule::fourAxisCommandCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, rpy_gain_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, RollPitchYawTerms),
                                  "rpy/gain", &rpy_gain_msg_, &FlightControlRosModule::rpyGainCallbackStatic_,
                                  ON_NEW_DATA);

  (void)init_subscription_default(
      node, p_matrix_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PMatrixPseudoInverseWithInertia),
      "p_matrix_pseudo_inverse_inertia", &p_matrix_msg_, &FlightControlRosModule::pMatrixCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, torque_allocation_sub_,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, TorqueAllocationMatrixInv),
                                  "torque_allocation_matrix_inv", &torque_allocation_msg_,
                                  &FlightControlRosModule::torqueAllocationCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, offset_rot_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, DesireCoord),
                                  "desire_coordinate", &offset_rot_msg_,
                                  &FlightControlRosModule::offsetRotCallbackStatic_, ON_NEW_DATA);

  (void)init_publisher_default(node, config_ack_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                               "flight_config_ack");

  (void)init_publisher_default(node, control_term_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, RollPitchYawTerms), "rpy/pid");

  (void)init_publisher_default(node, control_feedback_state_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, RollPitchYawTerm), "rpy/feedback_state");

  (void)init_service_default(node, att_control_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                             "set_attitude_control", &att_control_req_, &att_control_res_,
                             &FlightControlRosModule::attitudeControlCallbackStatic_);

  (void)init_service_default(node, position_control_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                             "set_position_control", &position_control_req_, &position_control_res_,
                             &FlightControlRosModule::positionControlCallbackStatic_);
}

void FlightControlRosModule::update()
{
  lock_control_();
  flight_control_.update();
  unlock_control_();
}

void FlightControlRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;

  lock_control_();

  uint8_t ack = 0;
  if (flight_control_.consumeConfigAck(ack))
  {
    config_ack_msg_.data = ack;
    (void)rcl_publish(&config_ack_pub_, &config_ack_msg_, nullptr);
  }

  AttitudeController &att = flight_control_.getAttitudeController();
  if (att.getUavModel() == FlightControlUavModel::DRAGON)
  {
    if (att.controlFeedbackStatePublishReady(true))
    {
      fillControlFeedback_(att.getControlFeedbackState());
      (void)rcl_publish(&control_feedback_state_pub_, &control_feedback_state_msg_, nullptr);
    }
  }
  else if (att.controlTermPublishReady(true))
  {
    fillControlTerms_(att.getControlTerms());
    (void)rcl_publish(&control_term_pub_, &control_term_msg_, nullptr);
  }

  unlock_control_();
}

void FlightControlRosModule::configure_message_storage_()
{
  four_axis_cmd_msg_.base_thrust.data = four_axis_base_thrust_buf_;
  four_axis_cmd_msg_.base_thrust.size = 0;
  four_axis_cmd_msg_.base_thrust.capacity = MAX_FOUR_AXIS_BASE_THRUST_SIZE;

  rpy_gain_msg_.motors.data = rpy_gain_buf_;
  rpy_gain_msg_.motors.size = 0;
  rpy_gain_msg_.motors.capacity = MAX_RPY_TERMS_SIZE;

  p_matrix_msg_.pseudo_inverse.data = p_matrix_buf_;
  p_matrix_msg_.pseudo_inverse.size = 0;
  p_matrix_msg_.pseudo_inverse.capacity = MAX_P_MATRIX_SIZE;

  torque_allocation_msg_.rows.data = torque_allocation_buf_;
  torque_allocation_msg_.rows.size = 0;
  torque_allocation_msg_.rows.capacity = MAX_TORQUE_ALLOC_SIZE;

  position_config_msg_.vertical_acceleration_to_thrust.data = position_thrust_conversion_buf_;
  position_config_msg_.vertical_acceleration_to_thrust.size = 0;
  position_config_msg_.vertical_acceleration_to_thrust.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.yaw_acceleration_to_thrust.data = position_yaw_acceleration_conversion_buf_;
  position_config_msg_.yaw_acceleration_to_thrust.size = 0;
  position_config_msg_.yaw_acceleration_to_thrust.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.z_p_gain.data = position_z_p_gain_buf_;
  position_config_msg_.z_p_gain.size = 0;
  position_config_msg_.z_p_gain.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.z_i_gain.data = position_z_i_gain_buf_;
  position_config_msg_.z_i_gain.size = 0;
  position_config_msg_.z_i_gain.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.z_d_gain.data = position_z_d_gain_buf_;
  position_config_msg_.z_d_gain.size = 0;
  position_config_msg_.z_d_gain.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.yaw_p_gain.data = position_yaw_p_gain_buf_;
  position_config_msg_.yaw_p_gain.size = 0;
  position_config_msg_.yaw_p_gain.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.yaw_i_gain.data = position_yaw_i_gain_buf_;
  position_config_msg_.yaw_i_gain.size = 0;
  position_config_msg_.yaw_i_gain.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  position_config_msg_.yaw_d_gain.data = position_yaw_d_gain_buf_;
  position_config_msg_.yaw_d_gain.size = 0;
  position_config_msg_.yaw_d_gain.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM;

  control_term_msg_.motors.data = control_term_buf_;
  control_term_msg_.motors.size = 0;
  control_term_msg_.motors.capacity = MAX_RPY_TERMS_SIZE;
}

void FlightControlRosModule::fillControlTerms_(const FlightControlRpyTerms &src)
{
  const size_t n = src.motors_count > MAX_RPY_TERMS_SIZE ? MAX_RPY_TERMS_SIZE : src.motors_count;
  control_term_msg_.motors.size = n;

  for (size_t i = 0; i < n; ++i)
  {
    control_term_buf_[i].roll_p = src.motors[i].roll_p;
    control_term_buf_[i].roll_i = src.motors[i].roll_i;
    control_term_buf_[i].roll_d = src.motors[i].roll_d;
    control_term_buf_[i].pitch_p = src.motors[i].pitch_p;
    control_term_buf_[i].pitch_i = src.motors[i].pitch_i;
    control_term_buf_[i].pitch_d = src.motors[i].pitch_d;
    control_term_buf_[i].yaw_d = src.motors[i].yaw_d;
  }
}

void FlightControlRosModule::fillControlFeedback_(const FlightControlRpyTerm &src)
{
  control_feedback_state_msg_.roll_p = src.roll_p;
  control_feedback_state_msg_.roll_i = src.roll_i;
  control_feedback_state_msg_.roll_d = src.roll_d;
  control_feedback_state_msg_.pitch_p = src.pitch_p;
  control_feedback_state_msg_.pitch_i = src.pitch_i;
  control_feedback_state_msg_.pitch_d = src.pitch_d;
  control_feedback_state_msg_.yaw_d = src.yaw_d;
}

void FlightControlRosModule::lock_control_()
{
  if (control_mutex_ != nullptr && *control_mutex_ != nullptr)
  {
    osMutexWait(*control_mutex_, osWaitForever);
  }
}

void FlightControlRosModule::unlock_control_()
{
  if (control_mutex_ != nullptr && *control_mutex_ != nullptr)
  {
    osMutexRelease(*control_mutex_);
  }
}

void FlightControlRosModule::flightConfigCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__FlightConfigCmd *>(msgin);

  instance_->lock_control_();
  (void)instance_->flight_control_.applyFlightConfig(msg->cmd);
  instance_->unlock_control_();
}

void FlightControlRosModule::uavInfoCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__UavInfo *>(msgin);

  instance_->lock_control_();
  instance_->flight_control_.applyUavInfo(msg->motor_num, static_cast<int8_t>(msg->uav_model));
  instance_->unlock_control_();
}

void FlightControlRosModule::gimbalDofCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const std_msgs__msg__UInt8 *>(msgin);

  instance_->lock_control_();
  instance_->flight_control_.applyGimbalDof(msg->data);
  instance_->unlock_control_();
}

void FlightControlRosModule::fourAxisCommandCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__FourAxisCommand *>(msgin);

  FlightControlFourAxisCommand cmd;
  cmd.angles[0] = msg->angles[0];
  cmd.angles[1] = msg->angles[1];
  cmd.angles[2] = msg->angles[2];
  cmd.base_thrust_count = msg->base_thrust.size > MAX_FOUR_AXIS_BASE_THRUST_SIZE ? MAX_FOUR_AXIS_BASE_THRUST_SIZE :
                                                                                   msg->base_thrust.size;
  for (size_t i = 0; i < cmd.base_thrust_count; ++i)
  {
    cmd.base_thrust[i] = msg->base_thrust.data[i];
  }

  instance_->lock_control_();
  (void)instance_->flight_control_.applyFourAxisCommand(cmd);
  instance_->unlock_control_();
}

void FlightControlRosModule::rpyGainCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__RollPitchYawTerms *>(msgin);

  FlightControlRpyTerms gains;
  gains.motors_count = msg->motors.size > MAX_RPY_TERMS_SIZE ? MAX_RPY_TERMS_SIZE : msg->motors.size;
  for (size_t i = 0; i < gains.motors_count; ++i)
  {
    const auto &src = msg->motors.data[i];
    gains.motors[i].roll_p = src.roll_p;
    gains.motors[i].roll_i = src.roll_i;
    gains.motors[i].roll_d = src.roll_d;
    gains.motors[i].pitch_p = src.pitch_p;
    gains.motors[i].pitch_i = src.pitch_i;
    gains.motors[i].pitch_d = src.pitch_d;
    gains.motors[i].yaw_d = src.yaw_d;
  }

  instance_->lock_control_();
  (void)instance_->flight_control_.applyRpyGains(gains);
  instance_->unlock_control_();
}

void FlightControlRosModule::pMatrixCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__PMatrixPseudoInverseWithInertia *>(msgin);

  FlightControlPMatrixPseudoInverseWithInertia dst;
  dst.pseudo_inverse_count = msg->pseudo_inverse.size > MAX_P_MATRIX_SIZE ? MAX_P_MATRIX_SIZE :
                                                                            msg->pseudo_inverse.size;
  for (size_t i = 0; i < dst.pseudo_inverse_count; ++i)
  {
    dst.pseudo_inverse[i].r = msg->pseudo_inverse.data[i].r;
    dst.pseudo_inverse[i].p = msg->pseudo_inverse.data[i].p;
    dst.pseudo_inverse[i].y = msg->pseudo_inverse.data[i].y;
  }
  for (size_t i = 0; i < 6; ++i)
  {
    dst.inertia[i] = msg->inertia[i];
  }

  instance_->lock_control_();
  (void)instance_->flight_control_.applyPMatrixInertia(dst);
  instance_->unlock_control_();
}

void FlightControlRosModule::torqueAllocationCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__TorqueAllocationMatrixInv *>(msgin);

  FlightControlTorqueAllocationMatrixInv dst;
  dst.rows_count = msg->rows.size > MAX_TORQUE_ALLOC_SIZE ? MAX_TORQUE_ALLOC_SIZE : msg->rows.size;
  for (size_t i = 0; i < dst.rows_count; ++i)
  {
    dst.rows[i].x = msg->rows.data[i].x;
    dst.rows[i].y = msg->rows.data[i].y;
    dst.rows[i].z = msg->rows.data[i].z;
  }

  instance_->lock_control_();
  (void)instance_->flight_control_.applyTorqueAllocationMatrixInv(dst);
  instance_->unlock_control_();
}

void FlightControlRosModule::offsetRotCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__DesireCoord *>(msgin);

  FlightControlDesireCoord dst;
  dst.roll = msg->roll;
  dst.pitch = msg->pitch;
  dst.yaw = msg->yaw;

  instance_->lock_control_();
  instance_->flight_control_.applyOffsetRotation(dst);
  instance_->unlock_control_();
}

void FlightControlRosModule::positionConfigCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__PositionControlConfig *>(msgin);
  const size_t motor_count = msg->vertical_acceleration_to_thrust.size;
  if (msg->use_lqi_gains &&
      (msg->yaw_acceleration_to_thrust.size != motor_count || msg->z_p_gain.size != motor_count ||
       msg->z_i_gain.size != motor_count || msg->z_d_gain.size != motor_count || msg->yaw_p_gain.size != motor_count ||
       msg->yaw_i_gain.size != motor_count || msg->yaw_d_gain.size != motor_count))
  {
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
  config.motor_count = motor_count > MAX_FLIGHT_CONTROL_MOTOR_NUM ? MAX_FLIGHT_CONTROL_MOTOR_NUM : motor_count;
  for (size_t motor = 0; motor < config.motor_count; ++motor)
  {
    config.vertical_acceleration_to_thrust[motor] = msg->vertical_acceleration_to_thrust.data[motor];
    if (motor < msg->yaw_acceleration_to_thrust.size)
      config.yaw_acceleration_to_thrust[motor] = msg->yaw_acceleration_to_thrust.data[motor];
    if (motor < msg->z_p_gain.size) config.z_p_gain[motor] = msg->z_p_gain.data[motor];
    if (motor < msg->z_i_gain.size) config.z_i_gain[motor] = msg->z_i_gain.data[motor];
    if (motor < msg->z_d_gain.size) config.z_d_gain[motor] = msg->z_d_gain.data[motor];
    if (motor < msg->yaw_p_gain.size) config.yaw_p_gain[motor] = msg->yaw_p_gain.data[motor];
    if (motor < msg->yaw_i_gain.size) config.yaw_i_gain[motor] = msg->yaw_i_gain.data[motor];
    if (motor < msg->yaw_d_gain.size) config.yaw_d_gain[motor] = msg->yaw_d_gain.data[motor];
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

  instance_->lock_control_();
  (void)instance_->flight_control_.applyPositionControlConfig(config);
  instance_->unlock_control_();
}

void FlightControlRosModule::positionSetpointCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__PositionControlSetpoint *>(msgin);
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

  instance_->lock_control_();
  (void)instance_->flight_control_.applyPositionControlSetpoint(setpoint);
  instance_->unlock_control_();
}

void FlightControlRosModule::attitudeControlCallbackStatic_(const void *req_msg, void *res_msg)
{
  if (instance_ == nullptr || req_msg == nullptr || res_msg == nullptr) return;
  const auto *req = reinterpret_cast<const std_srvs__srv__SetBool_Request *>(req_msg);
  auto *res = reinterpret_cast<std_srvs__srv__SetBool_Response *>(res_msg);

  instance_->lock_control_();
  instance_->flight_control_.setAttitudeControlFlag(req->data);
  instance_->unlock_control_();

  res->success = true;
}

void FlightControlRosModule::positionControlCallbackStatic_(const void *req_msg, void *res_msg)
{
  if (instance_ == nullptr || req_msg == nullptr || res_msg == nullptr) return;
  const auto *req = reinterpret_cast<const std_srvs__srv__SetBool_Request *>(req_msg);
  auto *res = reinterpret_cast<std_srvs__srv__SetBool_Response *>(res_msg);

  instance_->lock_control_();
  instance_->flight_control_.setPositionControlEnabled(req->data);
  res->success = instance_->flight_control_.getPositionController().enabled() == req->data;
  instance_->unlock_control_();
}

#endif  // !SIMULATION
