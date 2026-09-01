#include "flight_control/flight_control.h"

#include <cmath>

#ifndef SIMULATION
#include <map>
#endif

#ifdef SIMULATION
#include "thruster/simulation/thruster_manager.h"
#include <chrono>
#else
#include "servo/servo.h"
#include "thruster/board/thruster_manager.h"
#endif

void FlightControl::init(StateEstimate *estimator, ThrusterManager *thruster,
#ifndef SIMULATION
                         DirectServo *servo
#else
                         void *servo
#endif
)
{
#ifdef SIMULATION
  (void)servo;
#endif

  estimator_ = estimator;
  thruster_ = thruster;
#ifndef SIMULATION
  servo_ = servo;
#endif

  att_controller_.init(estimator_);
  position_controller_.reset();
  position_controller_.setEnabled(false);
  supervisor_.reset();
  start_control_flag_ = false;
  enabled_ = true;
  force_landing_flag_ = false;
  gimbal_set_flag_ = false;
  config_ack_pending_ = false;
  physical_motor_count_ = 0;
  previous_arming_state_ = FlightArmingState::DISARMED;
  previous_control_mode_ = FlightControlMode::IDLE;
  managed_setpoint_update_ms_ = 0U;
  last_external_setpoint_ms_ = 0U;
  has_external_position_setpoint_ = false;
  using_external_position_setpoint_ = false;
  rc_connected_ = false;
  rc_active_ = false;
  ros_link_state_ = FlightLinkState::UNKNOWN;
  network_link_state_ = FlightLinkState::UNKNOWN;
  last_network_heartbeat_ms_ = 0U;
  network_heartbeat_seen_ = false;
  last_control_update_ms_ = 0U;
  control_loop_period_ms_ = 0U;
  control_loop_period_valid_ = false;
  control_loop_sequence_ = 0U;
  observed_imu_timestamp_ms_ = 0U;
  last_imu_activity_ms_ = 0U;
  parameters_applied_ = false;
  rpy_gains_cached_ = false;
  p_matrix_cached_ = false;
  torque_allocation_cached_ = false;
  offset_rotation_cached_ = false;
  health_config_cached_ = false;

  loadParameterDatabase();
}

void FlightControl::update()
{
  if (!enabled_)
  {
    start_control_flag_ = false;
    force_landing_flag_ = false;
    if (thruster_ != nullptr)
    {
      const float zero_thrust = 0.0f;
      (void)thruster_->outputThrust(&zero_thrust, 0U, false);
    }
    return;
  }

  updateRuntimeHealthFacts_();
  updateLinkStates_();
  supervisor_.update(supervisorInput_());

  ThrusterControlLimits limits;
  if (thruster_ != nullptr)
  {
    limits = thruster_->getControlLimits();
  }

  att_controller_.setThrusterLimits(limits);

  synchronizeSupervisor_();
  updateManagedSetpoint_();

  if (!force_landing_flag_ && position_controller_.enabled() && position_controller_.active() && start_control_flag_ &&
      estimator_ != nullptr)
  {
    FlightControlFourAxisCommand position_command;
    const uint32_t now_ms = nowMillis_();
    if (!att_controller_.getIntegrateFlag() &&
        position_controller_.shouldEnableAttitudeIntegration(estimator_->outputState()))
    {
      att_controller_.setIntegrateFlag(true);
    }
    if (estimator_->positionControlStateValid() &&
        position_controller_.update(estimator_->outputState(), estimator_->stateValidity(), now_ms, position_command))
    {
      (void)att_controller_.applyFourAxisCommand(position_command);
    }
    else if (!force_landing_flag_)
    {
      supervisor_.reportControllerForceLand(supervisorInput_());
      synchronizeSupervisor_();
      setConfigAck_(FlightControlCommand::FORCE_LANDING_CMD);
    }
  }

  att_controller_.update();

  if (!force_landing_flag_ && att_controller_.getForceLandingFlag())
  {
    supervisor_.reportControllerForceLand(supervisorInput_());
    synchronizeSupervisor_();
    setConfigAck_(FlightControlCommand::FORCE_LANDING_CMD);
  }

  if (thruster_ != nullptr)
  {
    thruster_->setRotorDivider(att_controller_.getRotorDivider());
    (void)thruster_->outputThrust(att_controller_.getTargetThrust(), att_controller_.getThrusterCount(),
                                  start_control_flag_);
  }

  applyGimbalOutput_();
}

bool FlightControl::applyFlightConfig(uint8_t command)
{
  return requestFlightCommand(command, FlightCommandSource::ROS);
}

bool FlightControl::requestFlightCommand(uint8_t command, uint8_t source)
{
  if (!enabled_) return false;
  if (command == FlightControlCommand::INTEGRATION_CONTROL_ON_CMD)
  {
    att_controller_.setIntegrateFlag(true);
    return true;
  }
  if (command == FlightControlCommand::INTEGRATION_CONTROL_OFF_CMD)
  {
    att_controller_.setIntegrateFlag(false);
    return true;
  }

  ThrusterControlLimits limits;
  if (thruster_ != nullptr)
  {
    limits = thruster_->getControlLimits();
  }
  att_controller_.setThrusterLimits(limits);

  if (!supervisor_.request(command, source, supervisorInput_())) return false;

  switch (command)
  {
    case FlightControlCommand::ARM_ON_CMD: {
      initializeHoldSetpoint_();
      synchronizeSupervisor_();
      setConfigAck_(FlightControlCommand::ARM_ON_CMD);
      return true;
    }

    case FlightControlCommand::TAKEOFF_CMD:
      initializeTakeoffSetpoint_();
      synchronizeSupervisor_();
      setConfigAck_(FlightControlCommand::TAKEOFF_CMD);
      return true;

    case FlightControlCommand::LAND_CMD:
      initializeLandingSetpoint_();
      synchronizeSupervisor_();
      setConfigAck_(FlightControlCommand::LAND_CMD);
      return true;

    case FlightControlCommand::ARM_OFF_CMD:
    case FlightControlCommand::FORCE_LANDING_CMD:
    case FlightControlCommand::HALT_CMD:
      synchronizeSupervisor_();
      setConfigAck_(command);
      return true;

    default:
      break;
  }

  return false;
}

void FlightControl::applyUavInfo(uint8_t motor_num, int8_t uav_model)
{
  physical_motor_count_ = motor_num;
  att_controller_.setUavModel(uav_model);

  if (thruster_ != nullptr)
  {
    thruster_->setRotorDivider(att_controller_.getRotorDivider());
  }

  configureMotorCount_();
  refreshParameterStage_();
}

void FlightControl::applyGimbalDof(uint8_t gimbal_dof)
{
  if (gimbal_dof != 0 && !gimbal_set_flag_)
  {
    att_controller_.setGimbalDof(gimbal_dof);
    att_controller_.setRotorCoef(gimbal_dof + 1);
    gimbal_set_flag_ = true;
    configureMotorCount_();
    refreshParameterStage_();
  }
}

bool FlightControl::applyFourAxisCommand(const FlightControlFourAxisCommand &cmd)
{
  if (position_controller_.enabled()) return false;
  return att_controller_.applyFourAxisCommand(cmd);
}

bool FlightControl::applyPositionControlConfig(const PositionControlConfig &config)
{
  // Gain and limit changes preserve controller state. Structural changes are
  // still disarmed-only because they require resetting the controller.
  if (start_control_flag_ && !position_controller_.canReconfigureInFlight(config)) return false;
  if (!supervisor_.configure(config.supervisor)) return false;
  if (!position_controller_.configure(config)) return false;
  position_control_config_ = config;
  refreshParameterStage_();
  return true;
}

bool FlightControl::applyHealthConfig(const HealthManagerConfig &config)
{
  if (!supervisor_.configureHealth(config)) return false;
  health_config_ = config;
  health_config_cached_ = true;
  refreshParameterStage_();
  return true;
}

bool FlightControl::applyPositionControlSetpoint(const PositionControlSetpoint &setpoint)
{
  if (!position_controller_.configured()) return false;
  external_position_setpoint_ = setpoint;
  has_external_position_setpoint_ = true;
  last_external_setpoint_ms_ = nowMillis_();

  const FlightSupervisorStatus &status = supervisor_.status();
  if (status.arming_state == FlightArmingState::ARMED && status.flight_phase == FlightPhase::AIRBORNE &&
      status.control_mode == FlightControlMode::POSITION && setpoint.active)
  {
    PositionControlSetpoint accepted = setpoint;
    accepted.active = true;
    accepted.landing = false;
    position_controller_.setSetpoint(accepted, last_external_setpoint_ms_);
    using_external_position_setpoint_ = true;
    supervisor_.noteExternalSetpoint();
  }
  return true;
}

void FlightControl::applyPositionControlRcInput(const PositionControlRcInput &input)
{
  rc_connected_ = input.connected;
  const float threshold = position_control_config_.rc_deadzone;
  rc_active_ = input.connected && (std::fabs(input.lateral) > threshold || std::fabs(input.forward) > threshold ||
                                   std::fabs(input.vertical) > threshold || std::fabs(input.yaw) > threshold);
  position_controller_.setRcInput(input, nowMillis_());
}

void FlightControl::setRosLinkState(uint8_t state)
{
  if (state > FlightLinkState::DISABLED) state = FlightLinkState::UNKNOWN;
  ros_link_state_ = state;
}

void FlightControl::noteNetworkHeartbeat()
{
  last_network_heartbeat_ms_ = nowMillis_();
  network_heartbeat_seen_ = true;
  network_link_state_ = FlightLinkState::CONNECTED;
}

void FlightControl::setPositionControlEnabled(bool enabled)
{
  if (start_control_flag_) return;
  position_controller_.setEnabled(enabled);
  refreshParameterStage_();
}

bool FlightControl::positionControlStateValid() const
{
  return estimator_ != nullptr && estimator_->positionControlStateValid();
}

bool FlightControl::positionControlReady() const
{
  return estimator_ != nullptr && position_controller_.configured() && position_controller_.enabled() &&
         estimator_->positionControlStateValid();
}

FlightSupervisorInput FlightControl::supervisorInput_() const
{
  FlightSupervisorInput input;
  input.now_ms = nowMillis_();
  input.attitude_ready = att_controller_.activated();
  input.position_control_enabled = position_controller_.enabled();
  input.position_ready = positionControlReady();
  input.actuator_ready = thruster_ != nullptr && physical_motor_count_ > 0U && thruster_->getControlLimits().configured;
  input.config_ready = input.attitude_ready && (!input.position_control_enabled || position_controller_.configured());
  input.rc_connected = rc_connected_;
  input.ros_link_state = ros_link_state_;
  input.network_link_state = network_link_state_;
  input.rc_link_state = rc_connected_ ? FlightLinkState::CONNECTED : FlightLinkState::DISCONNECTED;
  input.rc_active = rc_active_;
  if (estimator_ != nullptr)
  {
    input.estimation_validity = estimator_->stateValidity();
    input.rejected_measurements = estimator_->outputRejectedMeasurementCount();
    input.estimation_ready = estimator_->outputInitialized() &&
                             (input.position_control_enabled ?
                                  input.position_ready :
                                  (input.estimation_validity & Eskf15::ATTITUDE_VALID) != 0U);
    const Eskf15::State &state = estimator_->outputState();
    input.vertical_position = state.position.z;
    input.vertical_velocity = state.velocity.z;
  }
  if (thruster_ != nullptr)
  {
    input.battery_voltage = thruster_->measuredBatteryVoltage();
    input.power_monitor_present = input.battery_voltage > 0.0f;
  }
  input.primary_imu_present = estimator_ != nullptr && estimator_->attitudeEnabled();
  input.last_primary_imu_update_ms = last_imu_activity_ms_;
  input.control_loop_period_valid = control_loop_period_valid_;
  input.control_loop_period_ms = control_loop_period_ms_;
  input.control_loop_sequence = control_loop_sequence_;
  return input;
}

void FlightControl::updateRuntimeHealthFacts_()
{
  ++control_loop_sequence_;
  const uint32_t now_ms = nowMillis_();
  if (last_control_update_ms_ != 0U)
  {
    control_loop_period_ms_ = now_ms - last_control_update_ms_;
    control_loop_period_valid_ = true;
  }
  last_control_update_ms_ = now_ms;

  if (estimator_ == nullptr) return;
  const uint32_t estimator_timestamp_ms = estimator_->lastImuUpdateTimeMs();
  if (estimator_timestamp_ms != 0U && estimator_timestamp_ms != observed_imu_timestamp_ms_)
  {
    observed_imu_timestamp_ms_ = estimator_timestamp_ms;
    last_imu_activity_ms_ = now_ms;
  }
}

void FlightControl::updateLinkStates_()
{
  if (!network_heartbeat_seen_)
  {
    network_link_state_ = FlightLinkState::UNKNOWN;
    return;
  }

  const uint32_t age_ms = nowMillis_() - last_network_heartbeat_ms_;
  if (age_ms > NETWORK_DISCONNECTED_TIMEOUT_MS)
    network_link_state_ = FlightLinkState::DISCONNECTED;
  else if (age_ms > NETWORK_STALE_TIMEOUT_MS)
    network_link_state_ = FlightLinkState::STALE;
  else
    network_link_state_ = FlightLinkState::CONNECTED;
}

void FlightControl::synchronizeSupervisor_()
{
  const FlightSupervisorStatus &status = supervisor_.status();
  const bool armed = status.arming_state == FlightArmingState::ARMED;
  const bool force_land = status.failsafe == FlightFailsafeState::FORCE_LAND;

  if (previous_arming_state_ != status.arming_state)
  {
    start_control_flag_ = armed;
    att_controller_.setStartControlFlag(armed);
    if (!armed) position_controller_.reset();
  }

  force_landing_flag_ = force_land;
  att_controller_.setForceLandingFlag(force_land);

  if (previous_control_mode_ != status.control_mode)
  {
    if (status.control_mode == FlightControlMode::POSITION && previous_control_mode_ == FlightControlMode::TAKEOFF)
    {
      managed_position_setpoint_.active = true;
      managed_position_setpoint_.landing = false;
      managed_position_setpoint_.manual_control_allowed = true;
      position_controller_.setSetpoint(managed_position_setpoint_, nowMillis_());
    }
    managed_setpoint_update_ms_ = nowMillis_();
  }

  previous_arming_state_ = status.arming_state;
  previous_control_mode_ = status.control_mode;
}

void FlightControl::initializeHoldSetpoint_()
{
  managed_position_setpoint_ = PositionControlSetpoint();
  if (estimator_ != nullptr)
  {
    const Eskf15::State &state = estimator_->outputState();
    managed_position_setpoint_.position = state.position;
    managed_position_setpoint_.yaw = state.attitude.get_euler_yaw();
    managed_position_setpoint_.initial_height = state.position.z;
  }
  managed_position_setpoint_.active = false;
  using_external_position_setpoint_ = false;
  managed_setpoint_update_ms_ = nowMillis_();
  position_controller_.setSetpoint(managed_position_setpoint_, managed_setpoint_update_ms_);
}

void FlightControl::initializeTakeoffSetpoint_()
{
  initializeHoldSetpoint_();
  managed_position_setpoint_.position.z = supervisor_.status().takeoff_target_height;
  managed_position_setpoint_.active = true;
  managed_position_setpoint_.manual_control_allowed = false;
  managed_position_setpoint_.landing = false;
  managed_setpoint_update_ms_ = nowMillis_();
  position_controller_.setSetpoint(managed_position_setpoint_, managed_setpoint_update_ms_);
}

void FlightControl::initializeLandingSetpoint_()
{
  managed_position_setpoint_ = position_controller_.setpoint();
  if (estimator_ != nullptr)
  {
    const Eskf15::State &state = estimator_->outputState();
    managed_position_setpoint_.position.z = state.position.z;
  }
  managed_position_setpoint_.velocity.zero();
  managed_position_setpoint_.acceleration.zero();
  managed_position_setpoint_.active = true;
  managed_position_setpoint_.manual_control_allowed = false;
  managed_position_setpoint_.landing = true;
  managed_setpoint_update_ms_ = nowMillis_();
  position_controller_.setSetpoint(managed_position_setpoint_, managed_setpoint_update_ms_);
}

void FlightControl::updateManagedSetpoint_()
{
  const FlightSupervisorStatus &status = supervisor_.status();
  if (status.arming_state != FlightArmingState::ARMED || !position_controller_.enabled()) return;

  const uint32_t now_ms = nowMillis_();
  if (status.control_mode == FlightControlMode::TAKEOFF)
  {
    managed_position_setpoint_.position.z = status.takeoff_target_height;
    managed_position_setpoint_.active = true;
    managed_position_setpoint_.manual_control_allowed = false;
    managed_position_setpoint_.landing = false;
    position_controller_.setSetpoint(managed_position_setpoint_, now_ms);
    using_external_position_setpoint_ = false;
  }
  else if (status.control_mode == FlightControlMode::LAND)
  {
    float dt = static_cast<float>(now_ms - managed_setpoint_update_ms_) * 0.001f;
    if (!std::isfinite(dt) || dt < 0.0f || dt > 0.1f) dt = 0.0f;
    managed_position_setpoint_.position.z += supervisor_.config().landing_speed * dt;
    if (managed_position_setpoint_.position.z < status.takeoff_reference_height)
      managed_position_setpoint_.position.z = status.takeoff_reference_height;
    managed_position_setpoint_.velocity.z = supervisor_.config().landing_speed;
    managed_position_setpoint_.active = true;
    managed_position_setpoint_.manual_control_allowed = false;
    managed_position_setpoint_.landing = true;
    position_controller_.setSetpoint(managed_position_setpoint_, now_ms);
    using_external_position_setpoint_ = false;
  }
  else if (status.control_mode == FlightControlMode::POSITION)
  {
    if (status.flight_phase != FlightPhase::AIRBORNE)
    {
      managed_position_setpoint_.active = false;
      position_controller_.setSetpoint(managed_position_setpoint_, now_ms);
      managed_setpoint_update_ms_ = now_ms;
      return;
    }
    const bool external_fresh = has_external_position_setpoint_ && external_position_setpoint_.active &&
                                static_cast<uint32_t>(now_ms - last_external_setpoint_ms_) <=
                                    position_control_config_.setpoint_timeout_ms;
    if (!external_fresh)
    {
      if (estimator_ != nullptr && using_external_position_setpoint_)
      {
        const Eskf15::State &state = estimator_->outputState();
        managed_position_setpoint_.position = state.position;
        managed_position_setpoint_.yaw = state.attitude.get_euler_yaw();
        PositionControlSetpoint reset_offset_setpoint = managed_position_setpoint_;
        reset_offset_setpoint.active = false;
        position_controller_.setSetpoint(reset_offset_setpoint, now_ms);
      }
      managed_position_setpoint_.velocity.zero();
      managed_position_setpoint_.acceleration.zero();
      managed_position_setpoint_.active = true;
      managed_position_setpoint_.manual_control_allowed = true;
      managed_position_setpoint_.landing = false;
      position_controller_.setSetpoint(managed_position_setpoint_, now_ms);
      using_external_position_setpoint_ = false;
    }
  }
  managed_setpoint_update_ms_ = now_ms;
}

bool FlightControl::applyRpyGains(const FlightControlRpyTerms &gains)
{
  if (!att_controller_.applyRpyGains(gains)) return false;
  rpy_gains_config_ = gains;
  rpy_gains_cached_ = true;
  refreshParameterStage_();
  return true;
}

bool FlightControl::applyPMatrixInertia(const FlightControlPMatrixPseudoInverseWithInertia &msg)
{
  if (!att_controller_.applyPMatrixInertia(msg)) return false;
  p_matrix_config_ = msg;
  p_matrix_cached_ = true;
  refreshParameterStage_();
  return true;
}

bool FlightControl::applyTorqueAllocationMatrixInv(const FlightControlTorqueAllocationMatrixInv &msg)
{
  if (!att_controller_.applyTorqueAllocationMatrixInv(msg)) return false;
  torque_allocation_config_ = msg;
  torque_allocation_cached_ = true;
  refreshParameterStage_();
  return true;
}

void FlightControl::applyOffsetRotation(const FlightControlDesireCoord &msg)
{
  att_controller_.applyOffsetRotation(msg);
  offset_rotation_config_ = msg;
  offset_rotation_cached_ = true;
  refreshParameterStage_();
}

void FlightControl::setAttitudeControlFlag(bool flag)
{
  att_controller_.setAttitudeControlFlag(flag);
  refreshParameterStage_();
}

void FlightControl::loadParameterDatabase()
{
  parameter_database_.load();
  parameters_applied_ = parameter_database_.valid() && applyStoredParameters_();
}

bool FlightControl::prepareParameterCommit()
{
  if (supervisor_.status().arming_state == FlightArmingState::ARMED) return false;
  refreshParameterStage_();
  if (!parameter_database_.prepareCommit()) return false;
  parameters_applied_ = true;
  return true;
}

bool FlightControl::reloadParameterDatabase()
{
  if (supervisor_.status().arming_state == FlightArmingState::ARMED) return false;
  loadParameterDatabase();
  return parameters_applied_;
}

void FlightControl::refreshParameterStage_()
{
  FlightParameterPayload payload;
  payload.position_control_enabled = position_controller_.enabled() ? 1U : 0U;
  payload.attitude_control_enabled = att_controller_.getAttitudeControlFlag() ? 1U : 0U;

  if (physical_motor_count_ > 0U && att_controller_.getUavModel() >= FlightControlUavModel::DRONE)
  {
    payload.valid_fields |= FlightParameterField::AIRFRAME;
    payload.motor_count = physical_motor_count_;
    payload.uav_model = att_controller_.getUavModel();
    payload.gimbal_dof = att_controller_.getGimbalDof();
  }

  if (thruster_ != nullptr && thruster_->getPwmInfo(payload.pwm)) payload.valid_fields |= FlightParameterField::PWM;
  if (rpy_gains_cached_)
  {
    payload.valid_fields |= FlightParameterField::ATTITUDE_GAINS;
    payload.attitude_gains = rpy_gains_config_;
  }
  if (p_matrix_cached_)
  {
    payload.valid_fields |= FlightParameterField::P_MATRIX;
    payload.p_matrix = p_matrix_config_;
  }
  if (torque_allocation_cached_)
  {
    payload.valid_fields |= FlightParameterField::TORQUE_ALLOCATION;
    payload.torque_allocation = torque_allocation_config_;
  }
  if (offset_rotation_cached_)
  {
    payload.valid_fields |= FlightParameterField::OFFSET_ROTATION;
    payload.offset_rotation = offset_rotation_config_;
  }
  if (position_controller_.configured())
  {
    payload.valid_fields |= FlightParameterField::POSITION_CONTROL;
    payload.position_control = position_control_config_;
  }
  if (health_config_cached_)
  {
    payload.valid_fields |= FlightParameterField::HEALTH;
    payload.health = health_config_;
  }

  (void)parameter_database_.stage(payload);
}

bool FlightControl::applyStoredParameters_()
{
  if (!parameter_database_.valid()) return false;
  const FlightParameterPayload payload = parameter_database_.payload();
  bool success = true;

  applyUavInfo(payload.motor_count, payload.uav_model);
  if (payload.gimbal_dof > 0U) applyGimbalDof(payload.gimbal_dof);
  if (thruster_ == nullptr || !thruster_->applyPwmInfo(payload.pwm)) success = false;
  if ((payload.valid_fields & FlightParameterField::P_MATRIX) != 0U && !applyPMatrixInertia(payload.p_matrix))
    success = false;
  if ((payload.valid_fields & FlightParameterField::TORQUE_ALLOCATION) != 0U &&
      !applyTorqueAllocationMatrixInv(payload.torque_allocation))
    success = false;
  if (!applyRpyGains(payload.attitude_gains)) success = false;
  if ((payload.valid_fields & FlightParameterField::OFFSET_ROTATION) != 0U)
    applyOffsetRotation(payload.offset_rotation);
  if ((payload.valid_fields & FlightParameterField::POSITION_CONTROL) != 0U &&
      !applyPositionControlConfig(payload.position_control))
    success = false;
  if ((payload.valid_fields & FlightParameterField::HEALTH) != 0U && !applyHealthConfig(payload.health))
    success = false;

  setAttitudeControlFlag(payload.attitude_control_enabled != 0U);
  setPositionControlEnabled(payload.position_control_enabled != 0U);
  refreshParameterStage_();
  return success;
}

bool FlightControl::consumeConfigAck(uint8_t &ack)
{
  if (!config_ack_pending_) return false;
  ack = config_ack_;
  config_ack_pending_ = false;
  return true;
}

void FlightControl::configureMotorCount_()
{
  if (physical_motor_count_ == 0) return;

  uint16_t allocation_count = static_cast<uint16_t>(physical_motor_count_) *
                              static_cast<uint16_t>(att_controller_.getRotorCoef());
  if (allocation_count > MAX_FLIGHT_CONTROL_MOTOR_NUM)
  {
    allocation_count = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  }

  att_controller_.setMotorNumber(allocation_count);
}

void FlightControl::setConfigAck_(uint8_t ack)
{
  config_ack_ = ack;
  config_ack_pending_ = true;
}

void FlightControl::applyGimbalOutput_()
{
#ifndef SIMULATION
  if (servo_ == nullptr || !servo_->connected()) return;

  const uint8_t gimbal_dof = att_controller_.getGimbalDof();
  if (gimbal_dof == 0) return;

  const uint16_t thruster_count = att_controller_.getThrusterCount();
  const float *target_gimbal_angles = att_controller_.getTargetGimbalAngles();

  std::map<uint8_t, float> gimbal_map;

  if (gimbal_dof == 2)
  {
    for (uint16_t i = 0; i < thruster_count; ++i)
    {
      gimbal_map[static_cast<uint8_t>(2 * i)] = start_control_flag_ ? target_gimbal_angles[2 * i] : 0.0f;
      gimbal_map[static_cast<uint8_t>(2 * i + 1)] = start_control_flag_ ? target_gimbal_angles[2 * i + 1] : 0.0f;
    }
  }
  else if (gimbal_dof == 1)
  {
    for (uint16_t i = 0; i < thruster_count; ++i)
    {
      gimbal_map[static_cast<uint8_t>(i)] = start_control_flag_ ? target_gimbal_angles[i] : 0.0f;
    }
  }

  if (gimbal_map.empty()) return;

  if (start_control_flag_)
  {
    servo_->setGoalAngle(gimbal_map, ValueType::RADIAN);
  }
  else
  {
    servo_->torqueEnable(gimbal_map);
  }
#endif
}

uint32_t FlightControl::nowMillis_()
{
#ifdef SIMULATION
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
#else
  return HAL_GetTick();
#endif
}
