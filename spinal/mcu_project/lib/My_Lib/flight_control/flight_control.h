#pragma once

#include <cstddef>
#include <cstdint>

#include "flight_control/attitude/attitude_control.h"
#include "flight_control/position/position_control.h"
#include "flight_control/supervisor/flight_supervisor.h"

class StateEstimate;
class ThrusterManager;

#ifndef SIMULATION
class DirectServo;
#endif

class FlightControl
{
public:
  FlightControl() = default;
  ~FlightControl() = default;

  void init(StateEstimate *estimator, ThrusterManager *thruster,
#ifndef SIMULATION
            DirectServo *servo = nullptr
#else
            void *servo = nullptr
#endif
  );

  void update();

  bool applyFlightConfig(uint8_t command);
  bool requestFlightCommand(uint8_t command, uint8_t source);
  void applyUavInfo(uint8_t motor_num, int8_t uav_model);
  void applyGimbalDof(uint8_t gimbal_dof);
  bool applyFourAxisCommand(const FlightControlFourAxisCommand &cmd);
  bool applyPositionControlConfig(const PositionControlConfig &config);
  bool applyHealthConfig(const HealthManagerConfig &config);
  bool applyPositionControlSetpoint(const PositionControlSetpoint &setpoint);
  void applyPositionControlRcInput(const PositionControlRcInput &input);
  void setRosLinkState(uint8_t state);
  void noteNetworkHeartbeat();
  void setPositionControlEnabled(bool enabled);
  bool applyRpyGains(const FlightControlRpyTerms &gains);
  bool applyPMatrixInertia(const FlightControlPMatrixPseudoInverseWithInertia &msg);
  bool applyTorqueAllocationMatrixInv(const FlightControlTorqueAllocationMatrixInv &msg);
  void applyOffsetRotation(const FlightControlDesireCoord &msg);
  void setAttitudeControlFlag(bool flag);

  bool consumeConfigAck(uint8_t &ack);

  AttitudeController &getAttitudeController() { return att_controller_; }
  const AttitudeController &getAttitudeController() const { return att_controller_; }
  PositionController &getPositionController() { return position_controller_; }
  const PositionController &getPositionController() const { return position_controller_; }
  FlightSupervisor &getSupervisor() { return supervisor_; }
  const FlightSupervisor &getSupervisor() const { return supervisor_; }

  bool startControl() const { return start_control_flag_; }
  bool forceLanding() const { return force_landing_flag_; }
  uint8_t physicalMotorCount() const { return physical_motor_count_; }
  bool positionControlStateValid() const;
  bool positionControlReady() const;

private:
  StateEstimate *estimator_{ nullptr };
  ThrusterManager *thruster_{ nullptr };

#ifndef SIMULATION
  DirectServo *servo_{ nullptr };
#endif

  AttitudeController att_controller_;
  PositionController position_controller_;
  FlightSupervisor supervisor_;
  PositionControlConfig position_control_config_{};
  PositionControlSetpoint external_position_setpoint_{};
  PositionControlSetpoint managed_position_setpoint_{};

  bool start_control_flag_{ false };
  bool force_landing_flag_{ false };
  bool gimbal_set_flag_{ false };
  bool config_ack_pending_{ false };
  uint8_t config_ack_{ 0 };
  uint8_t physical_motor_count_{ 0 };
  uint8_t previous_arming_state_{ FlightArmingState::DISARMED };
  uint8_t previous_control_mode_{ FlightControlMode::IDLE };
  uint32_t managed_setpoint_update_ms_{ 0U };
  uint32_t last_external_setpoint_ms_{ 0U };
  bool has_external_position_setpoint_{ false };
  bool using_external_position_setpoint_{ false };
  bool rc_connected_{ false };
  bool rc_active_{ false };
  uint8_t ros_link_state_{ FlightLinkState::UNKNOWN };
  uint8_t network_link_state_{ FlightLinkState::UNKNOWN };
  uint32_t last_network_heartbeat_ms_{ 0U };
  bool network_heartbeat_seen_{ false };
  uint32_t last_control_update_ms_{ 0U };
  uint32_t control_loop_period_ms_{ 0U };
  bool control_loop_period_valid_{ false };
  uint32_t control_loop_sequence_{ 0U };
  uint32_t observed_imu_timestamp_ms_{ 0U };
  uint32_t last_imu_activity_ms_{ 0U };

  static constexpr uint32_t NETWORK_STALE_TIMEOUT_MS = 1000U;
  static constexpr uint32_t NETWORK_DISCONNECTED_TIMEOUT_MS = 5000U;

  void configureMotorCount_();
  void setConfigAck_(uint8_t ack);
  void applyGimbalOutput_();
  FlightSupervisorInput supervisorInput_() const;
  void synchronizeSupervisor_();
  void initializeHoldSetpoint_();
  void initializeTakeoffSetpoint_();
  void initializeLandingSetpoint_();
  void updateManagedSetpoint_();
  void updateLinkStates_();
  void updateRuntimeHealthFacts_();
  static uint32_t nowMillis_();
};
