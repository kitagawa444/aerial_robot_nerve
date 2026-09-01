#ifndef SIMULATION

#include "flight_control/board/flight_control_ros_module.h"

#include "flight_control/flight_control_ros_adapter.h"
#include "flashmemory/flashmemory.h"
#include "flashmemory/application_capabilities.h"
#include "flashmemory/config_flash_storage.h"

#include <rmw_microros/rmw_microros.h>

FlightControlRosModule *FlightControlRosModule::instance_ = nullptr;

void FlightControlRosModule::init_hw(StateEstimate *estimator, ThrusterManager *thruster, DirectServo *servo,
                                     osMutexId *control_mutex, ConfigFlashDatabase *config_flash_database)
{
  control_mutex_ = control_mutex;
  config_flash_database_ = config_flash_database;
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
  spinal_msgs__msg__HealthConfig__init(&health_config_msg_);
  spinal_msgs__msg__PositionControlSetpoint__init(&position_setpoint_msg_);
  std_msgs__msg__Empty__init(&network_heartbeat_msg_);
  std_msgs__msg__UInt8__init(&config_ack_msg_);
  spinal_msgs__msg__FlightStatus__init(&flight_status_msg_);
  spinal_msgs__msg__RollPitchYawTerms__init(&control_term_msg_);
  spinal_msgs__msg__RollPitchYawTerm__init(&control_feedback_state_msg_);
  spinal_msgs__msg__ApplicationCapabilities__init(&application_capabilities_msg_);
  spinal_msgs__msg__ConfigFlashStatus__init(&config_flash_status_msg_);
  spinal_msgs__msg__FlightParameterTable__init(&flight_parameter_table_msg_);
  std_srvs__srv__SetBool_Request__init(&att_control_req_);
  std_srvs__srv__SetBool_Response__init(&att_control_res_);
  std_srvs__srv__SetBool_Request__init(&position_control_req_);
  std_srvs__srv__SetBool_Response__init(&position_control_res_);
  spinal_msgs__srv__ManageFlightParameters_Request__init(&parameter_database_req_);
  spinal_msgs__srv__ManageFlightParameters_Response__init(&parameter_database_res_);
  spinal_msgs__srv__ManageConfigFlash_Request__init(&config_flash_req_);
  spinal_msgs__srv__ManageConfigFlash_Response__init(&config_flash_res_);

  configure_message_storage_();

  (void)init_subscription_default(
      node, flight_config_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, FlightConfigCmd), "flight_config_cmd",
      &flight_config_msg_, &FlightControlRosModule::flightConfigCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, position_config_sub_,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PositionControlConfig),
                                  "position_control/config", &position_config_msg_,
                                  &FlightControlRosModule::positionConfigCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, health_config_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, HealthConfig),
                                  "health/config", &health_config_msg_,
                                  &FlightControlRosModule::healthConfigCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, position_setpoint_sub_,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PositionControlSetpoint),
                                  "position_control/setpoint", &position_setpoint_msg_,
                                  &FlightControlRosModule::positionSetpointCallbackStatic_, ON_NEW_DATA);

  (void)init_subscription_default(node, network_heartbeat_sub_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
                                  "network/heartbeat", &network_heartbeat_msg_,
                                  &FlightControlRosModule::networkHeartbeatCallbackStatic_, ON_NEW_DATA);

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

  (void)init_publisher_default(node, flight_status_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, FlightStatus),
                               "fc/flight_status");

  (void)init_publisher_default(node, control_term_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, RollPitchYawTerms), "rpy/pid");

  (void)init_publisher_default(node, control_feedback_state_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, RollPitchYawTerm), "rpy/feedback_state");

  (void)init_publisher_default(node, application_capabilities_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ApplicationCapabilities),
                               "fc/application_capabilities");

  (void)init_publisher_default(node, config_flash_status_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ConfigFlashStatus),
                               "fc/config_flash/status");

  (void)init_publisher_default(node, flight_parameter_table_pub_,
                               ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, FlightParameterTable),
                               "fc/flight_parameters/table");

  (void)init_service_default(node, att_control_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                             "set_attitude_control", &att_control_req_, &att_control_res_,
                             &FlightControlRosModule::attitudeControlCallbackStatic_);

  (void)init_service_default(node, position_control_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                             "set_position_control", &position_control_req_, &position_control_res_,
                             &FlightControlRosModule::positionControlCallbackStatic_);

  (void)init_service_default(node, parameter_database_srv_,
                             ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, ManageFlightParameters), "fc/parameters",
                             &parameter_database_req_, &parameter_database_res_,
                             &FlightControlRosModule::parameterDatabaseCallbackStatic_);

  (void)init_service_default(node, config_flash_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, ManageConfigFlash),
                             "fc/config_flash", &config_flash_req_, &config_flash_res_,
                             &FlightControlRosModule::configFlashCallbackStatic_);
}

void FlightControlRosModule::update()
{
  lock_control_();
  if (ros_ready_ == nullptr)
    flight_control_.setRosLinkState(FlightLinkState::CONNECTING);
  else
    flight_control_.setRosLinkState(ros_ready_->load(std::memory_order_acquire) ? FlightLinkState::CONNECTED :
                                                                                  FlightLinkState::DISCONNECTED);
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

  const FlightSupervisorStatus &status = flight_control_.getSupervisor().status();
  const uint32_t now_ms = HAL_GetTick();
  if (status.sequence != last_flight_status_sequence_ || now_ms - last_flight_status_publish_ms_ >= 100U)
  {
    const uint64_t epoch_ms = rmw_uros_epoch_millis();
    flight_status_msg_.stamp.sec = static_cast<int32_t>(epoch_ms / 1000ULL);
    flight_status_msg_.stamp.nanosec = static_cast<uint32_t>((epoch_ms % 1000ULL) * 1000000ULL);
    flight_control_ros::fillFlightStatus(flight_status_msg_, status);
    (void)rcl_publish(&flight_status_pub_, &flight_status_msg_, nullptr);
    last_flight_status_sequence_ = status.sequence;
    last_flight_status_publish_ms_ = now_ms;
  }

  if (now_ms - last_storage_status_publish_ms_ >= 1000U)
  {
    application_capabilities_msg_.board = spinal_msgs__msg__ApplicationCapabilities__BOARD_STM32H7_V2;
    application_capabilities_msg_.capability_mask = ApplicationCapability::mask();
    application_capabilities_msg_.max_motor_count = MAX_FLIGHT_CONTROL_MOTOR_NUM;
    application_capabilities_msg_.config_schema_version = ConfigFlashDatabase::SCHEMA_VERSION;
    application_capabilities_msg_.flight_parameter_schema_version = FlightParameterDatabase::SCHEMA_VERSION;
    (void)rcl_publish(&application_capabilities_pub_, &application_capabilities_msg_, nullptr);
    fillConfigFlashStatus_(config_flash_status_msg_);
    (void)rcl_publish(&config_flash_status_pub_, &config_flash_status_msg_, nullptr);
    flight_control_ros::fillFlightParameterTable(flight_parameter_table_msg_, flight_control_.parameterDatabase(),
                                                 flight_control_.parametersApplied(), true);
    (void)rcl_publish(&flight_parameter_table_pub_, &flight_parameter_table_msg_, nullptr);
    last_storage_status_publish_ms_ = now_ms;
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

  flight_parameter_table_msg_.pwm.motor_info.data = flight_parameter_pwm_motor_info_buf_;
  flight_parameter_table_msg_.pwm.motor_info.size = 0;
  flight_parameter_table_msg_.pwm.motor_info.capacity = MAX_PWM_MOTOR_INFO_SIZE;
  flight_parameter_table_msg_.attitude_gains.motors.data = rpy_gain_buf_;
  flight_parameter_table_msg_.attitude_gains.motors.size = 0;
  flight_parameter_table_msg_.attitude_gains.motors.capacity = MAX_RPY_TERMS_SIZE;
  flight_parameter_table_msg_.p_matrix.pseudo_inverse.data = p_matrix_buf_;
  flight_parameter_table_msg_.p_matrix.pseudo_inverse.size = 0;
  flight_parameter_table_msg_.p_matrix.pseudo_inverse.capacity = MAX_P_MATRIX_SIZE;
  flight_parameter_table_msg_.torque_allocation.rows.data = torque_allocation_buf_;
  flight_parameter_table_msg_.torque_allocation.rows.size = 0;
  flight_parameter_table_msg_.torque_allocation.rows.capacity = MAX_TORQUE_ALLOC_SIZE;

#define CONFIGURE_TABLE_FLOAT_SEQUENCE(field, buffer)                                                                  \
  flight_parameter_table_msg_.position_control.field.data = buffer;                                                    \
  flight_parameter_table_msg_.position_control.field.size = 0;                                                         \
  flight_parameter_table_msg_.position_control.field.capacity = MAX_FLIGHT_CONTROL_MOTOR_NUM
  CONFIGURE_TABLE_FLOAT_SEQUENCE(vertical_acceleration_to_thrust, position_thrust_conversion_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(yaw_acceleration_to_thrust, position_yaw_acceleration_conversion_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(z_p_gain, position_z_p_gain_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(z_i_gain, position_z_i_gain_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(z_d_gain, position_z_d_gain_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(yaw_p_gain, position_yaw_p_gain_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(yaw_i_gain, position_yaw_i_gain_buf_);
  CONFIGURE_TABLE_FLOAT_SEQUENCE(yaw_d_gain, position_yaw_d_gain_buf_);
#undef CONFIGURE_TABLE_FLOAT_SEQUENCE
}

void FlightControlRosModule::fillControlTerms_(const FlightControlRpyTerms &src)
{
  const size_t n = src.motors_count > MAX_RPY_TERMS_SIZE ? MAX_RPY_TERMS_SIZE : src.motors_count;
  control_term_msg_.motors.size = n;

  for (size_t i = 0; i < n; ++i)
  {
    flight_control_ros::fillRollPitchYawTerm(control_term_buf_[i], src.motors[i]);
  }
}

void FlightControlRosModule::fillControlFeedback_(const FlightControlRpyTerm &src)
{
  flight_control_ros::fillRollPitchYawTerm(control_feedback_state_msg_, src);
}

void FlightControlRosModule::fillConfigFlashStatus_(spinal_msgs__msg__ConfigFlashStatus &msg) const
{
  if (config_flash_database_ == nullptr)
  {
    msg = spinal_msgs__msg__ConfigFlashStatus{};
    return;
  }
  msg.valid = config_flash_database_->valid();
  msg.dirty = config_flash_database_->dirty();
  msg.reboot_required = config_flash_database_->rebootRequired();
  msg.persistent_storage = true;
  msg.active_slot = config_flash_database_->activeSlot();
  msg.active_uart3_driver = config_flash_database_->activeUart3Driver();
  msg.pending_uart3_driver = config_flash_database_->pendingUart3Driver();
  const ConfigFlashPayload &active = config_flash_database_->activeConfiguration();
  const ConfigFlashPayload &pending = config_flash_database_->pendingConfiguration();
  msg.active_imu_driver = active.imu_driver;
  msg.pending_imu_driver = pending.imu_driver;
  msg.active_barometer_enabled = active.barometer_enabled != 0U;
  msg.pending_barometer_enabled = pending.barometer_enabled != 0U;
  msg.active_servo_driver = active.servo_driver;
  msg.pending_servo_driver = pending.servo_driver;
  msg.active_attitude_estimation_enabled = active.attitude_estimation_enabled != 0U;
  msg.pending_attitude_estimation_enabled = pending.attitude_estimation_enabled != 0U;
  msg.active_height_estimation_enabled = active.height_estimation_enabled != 0U;
  msg.pending_height_estimation_enabled = pending.height_estimation_enabled != 0U;
  msg.active_position_estimation_enabled = active.position_estimation_enabled != 0U;
  msg.pending_position_estimation_enabled = pending.position_estimation_enabled != 0U;
  msg.active_flight_control_enabled = active.flight_control_enabled != 0U;
  msg.pending_flight_control_enabled = pending.flight_control_enabled != 0U;
  msg.active_motor_output_driver = active.motor_output_driver;
  msg.pending_motor_output_driver = pending.motor_output_driver;
  msg.schema_version = config_flash_database_->schemaVersion();
  msg.generation = config_flash_database_->generation();
  msg.crc32 = config_flash_database_->crc32();
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

  instance_->lock_control_();
  (void)flight_control_ros::applyFourAxisCommand(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::rpyGainCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__RollPitchYawTerms *>(msgin);

  instance_->lock_control_();
  (void)flight_control_ros::applyRpyGains(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::pMatrixCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__PMatrixPseudoInverseWithInertia *>(msgin);

  instance_->lock_control_();
  (void)flight_control_ros::applyPMatrixInertia(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::torqueAllocationCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__TorqueAllocationMatrixInv *>(msgin);

  instance_->lock_control_();
  (void)flight_control_ros::applyTorqueAllocationMatrixInv(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::offsetRotCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__DesireCoord *>(msgin);

  instance_->lock_control_();
  flight_control_ros::applyOffsetRotation(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::positionConfigCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__PositionControlConfig *>(msgin);

  instance_->lock_control_();
  (void)flight_control_ros::applyPositionControlConfig(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::healthConfigCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__HealthConfig *>(msgin);
  instance_->lock_control_();
  (void)flight_control_ros::applyHealthConfig(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::positionSetpointCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__PositionControlSetpoint *>(msgin);

  instance_->lock_control_();
  (void)flight_control_ros::applyPositionControlSetpoint(instance_->flight_control_, *msg);
  instance_->unlock_control_();
}

void FlightControlRosModule::networkHeartbeatCallbackStatic_(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  instance_->lock_control_();
  instance_->flight_control_.noteNetworkHeartbeat();
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

void FlightControlRosModule::parameterDatabaseCallbackStatic_(const void *req_msg, void *res_msg)
{
  if (instance_ == nullptr || req_msg == nullptr || res_msg == nullptr) return;
  const auto *req = reinterpret_cast<const spinal_msgs__srv__ManageFlightParameters_Request *>(req_msg);
  auto *res = reinterpret_cast<spinal_msgs__srv__ManageFlightParameters_Response *>(res_msg);

  instance_->lock_control_();
  res->success = false;
  res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_INVALID_COMMAND;
  res->persistent_storage = true;

  const bool armed = instance_->flight_control_.getSupervisor().status().arming_state == FlightArmingState::ARMED;
  if (req->command == spinal_msgs__srv__ManageFlightParameters_Request__STATUS)
  {
    res->success = true;
    res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_OK;
  }
  else if (req->command == spinal_msgs__srv__ManageFlightParameters_Request__COMMIT)
  {
    if (armed)
    {
      res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_ARMED;
    }
    else if (!instance_->flight_control_.prepareParameterCommit())
    {
      res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_INCOMPLETE;
    }
    else
    {
      const HAL_StatusTypeDef erase_status = FlashMemory::erase();
      const HAL_StatusTypeDef write_status = erase_status == HAL_OK ? FlashMemory::write() : erase_status;
      const HAL_StatusTypeDef read_status = write_status == HAL_OK ? FlashMemory::read() : write_status;
      const bool verified = read_status == HAL_OK && instance_->flight_control_.reloadParameterDatabase();
      res->success = erase_status == HAL_OK && write_status == HAL_OK && verified;
      res->result = res->success ? spinal_msgs__srv__ManageFlightParameters_Response__RESULT_OK :
                                   spinal_msgs__srv__ManageFlightParameters_Response__RESULT_STORAGE_ERROR;
      if (!res->success) instance_->flight_control_.getParameterDatabase().markCommitFailed();
    }
  }
  else if (req->command == spinal_msgs__srv__ManageFlightParameters_Request__RELOAD)
  {
    if (armed)
    {
      res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_ARMED;
    }
    else if (FlashMemory::read() != HAL_OK || !instance_->flight_control_.reloadParameterDatabase())
    {
      res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_NO_VALID_DATABASE;
    }
    else
    {
      res->success = true;
      res->result = spinal_msgs__srv__ManageFlightParameters_Response__RESULT_OK;
    }
  }

  const FlightParameterDatabase &database = instance_->flight_control_.parameterDatabase();
  res->valid = database.valid();
  res->dirty = database.dirty();
  res->applied = instance_->flight_control_.parametersApplied();
  res->schema_version = database.schemaVersion();
  res->generation = database.generation();
  res->crc32 = database.crc32();
  res->valid_fields = database.validFields();
  instance_->unlock_control_();
}

void FlightControlRosModule::configFlashCallbackStatic_(const void *req_msg, void *res_msg)
{
  if (instance_ == nullptr || req_msg == nullptr || res_msg == nullptr) return;
  const auto *req = reinterpret_cast<const spinal_msgs__srv__ManageConfigFlash_Request *>(req_msg);
  auto *res = reinterpret_cast<spinal_msgs__srv__ManageConfigFlash_Response *>(res_msg);

  instance_->lock_control_();
  res->success = false;
  res->result = spinal_msgs__srv__ManageConfigFlash_Response__RESULT_INVALID_COMMAND;
  ConfigFlashDatabase *database = instance_->config_flash_database_;
  const bool armed = instance_->flight_control_.getSupervisor().status().arming_state == FlightArmingState::ARMED;

  if (database == nullptr)
  {
    res->result = spinal_msgs__srv__ManageConfigFlash_Response__RESULT_STORAGE_ERROR;
  }
  else if (req->command == spinal_msgs__srv__ManageConfigFlash_Request__STATUS)
  {
    res->success = true;
    res->result = spinal_msgs__srv__ManageConfigFlash_Response__RESULT_OK;
  }
  else if (armed)
  {
    res->result = spinal_msgs__srv__ManageConfigFlash_Response__RESULT_ARMED;
  }
  else if (req->command == spinal_msgs__srv__ManageConfigFlash_Request__STAGE)
  {
    ConfigFlashPayload pending = database->pendingConfiguration();
    pending.imu_driver = req->imu_driver;
    pending.barometer_enabled = req->barometer_enabled ? 1U : 0U;
    pending.uart3_driver = req->uart3_driver;
    pending.servo_driver = req->servo_driver;
    pending.attitude_estimation_enabled = req->attitude_estimation_enabled ? 1U : 0U;
    pending.height_estimation_enabled = req->height_estimation_enabled ? 1U : 0U;
    pending.position_estimation_enabled = req->position_estimation_enabled ? 1U : 0U;
    pending.flight_control_enabled = req->flight_control_enabled ? 1U : 0U;
    pending.motor_output_driver = req->motor_output_driver;
    res->success = database->stageConfiguration(pending, ApplicationCapability::support());
    res->result = res->success ? spinal_msgs__srv__ManageConfigFlash_Response__RESULT_OK :
                                 spinal_msgs__srv__ManageConfigFlash_Response__RESULT_UNSUPPORTED;
  }
  else if (req->command == spinal_msgs__srv__ManageConfigFlash_Request__COMMIT)
  {
    res->success = ConfigFlashStorage::commit(*database);
    res->result = res->success ? spinal_msgs__srv__ManageConfigFlash_Response__RESULT_OK :
                                 spinal_msgs__srv__ManageConfigFlash_Response__RESULT_STORAGE_ERROR;
  }
  else if (req->command == spinal_msgs__srv__ManageConfigFlash_Request__RELOAD)
  {
    res->success = ConfigFlashStorage::reload(*database);
    res->result = res->success ? spinal_msgs__srv__ManageConfigFlash_Response__RESULT_OK :
                                 spinal_msgs__srv__ManageConfigFlash_Response__RESULT_NO_VALID_CONFIG;
  }

  instance_->fillConfigFlashStatus_(res->status);
  instance_->unlock_control_();
}

#endif  // !SIMULATION
