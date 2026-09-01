#pragma once

#ifndef SIMULATION

#include <cstddef>
#include <cstdint>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int8.h>
#include <std_srvs/srv/set_bool.h>

#include <spinal_msgs/msg/desire_coord.h>
#include <spinal_msgs/msg/application_capabilities.h>
#include <spinal_msgs/msg/config_flash_status.h>
#include <spinal_msgs/msg/flight_config_cmd.h>
#include <spinal_msgs/msg/flight_status.h>
#include <spinal_msgs/msg/flight_parameter_table.h>
#include <spinal_msgs/msg/four_axis_command.h>
#include <spinal_msgs/msg/health_config.h>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_unit.h>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.h>
#include <spinal_msgs/msg/position_control_config.h>
#include <spinal_msgs/msg/position_control_setpoint.h>
#include <spinal_msgs/msg/roll_pitch_yaw_term.h>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.h>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.h>
#include <spinal_msgs/msg/uav_info.h>
#include <spinal_msgs/msg/vector3_int16.h>
#include <spinal_msgs/srv/manage_flight_parameters.h>
#include <spinal_msgs/srv/manage_config_flash.h>

#include <ros_utils/ros_module_base.hpp>

#include "flight_control/flight_control.h"
#include "flashmemory/config_flash_database.h"
#include "servo/servo.h"
#include "state_estimate/state_estimate.h"
#include "thruster/board/thruster_manager.h"

class FlightControlRosModule final : public RosModuleBase
{
public:
  FlightControlRosModule()
    : RosModuleBase(RosModuleEntityCapacity().max_subscriptions(12).max_publishers(7).max_services(4).max_timers(0))
  {
  }

  void init_hw(StateEstimate *estimator, ThrusterManager *thruster, DirectServo *servo = nullptr,
               osMutexId *control_mutex = nullptr, ConfigFlashDatabase *config_flash_database = nullptr);

  FlightControl *getFlightControlCore() { return &flight_control_; }

  void create_entities(rcl_node_t &node) override;
  void update() override;
  void publish() override;

private:
  static constexpr size_t MAX_FOUR_AXIS_BASE_THRUST_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_RPY_TERMS_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_P_MATRIX_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_TORQUE_ALLOC_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_PWM_MOTOR_INFO_SIZE = MAX_THRUSTER_MOTOR_INFO_NUM;

  FlightControl flight_control_;
  osMutexId *control_mutex_{ nullptr };
  ConfigFlashDatabase *config_flash_database_{ nullptr };

  rcl_subscription_t flight_config_sub_{};
  rcl_subscription_t uav_info_sub_{};
  rcl_subscription_t gimbal_dof_sub_{};
  rcl_subscription_t four_axis_cmd_sub_{};
  rcl_subscription_t rpy_gain_sub_{};
  rcl_subscription_t p_matrix_sub_{};
  rcl_subscription_t torque_allocation_sub_{};
  rcl_subscription_t offset_rot_sub_{};
  rcl_subscription_t position_config_sub_{};
  rcl_subscription_t health_config_sub_{};
  rcl_subscription_t position_setpoint_sub_{};
  rcl_subscription_t network_heartbeat_sub_{};

  rcl_publisher_t config_ack_pub_{};
  rcl_publisher_t flight_status_pub_{};
  rcl_publisher_t control_term_pub_{};
  rcl_publisher_t control_feedback_state_pub_{};
  rcl_publisher_t application_capabilities_pub_{};
  rcl_publisher_t config_flash_status_pub_{};
  rcl_publisher_t flight_parameter_table_pub_{};

  rcl_service_t att_control_srv_{};
  rcl_service_t position_control_srv_{};
  rcl_service_t parameter_database_srv_{};
  rcl_service_t config_flash_srv_{};

  spinal_msgs__msg__FlightConfigCmd flight_config_msg_{};
  spinal_msgs__msg__UavInfo uav_info_msg_{};
  std_msgs__msg__UInt8 gimbal_dof_msg_{};

  spinal_msgs__msg__FourAxisCommand four_axis_cmd_msg_{};
  float four_axis_base_thrust_buf_[MAX_FOUR_AXIS_BASE_THRUST_SIZE]{};

  spinal_msgs__msg__RollPitchYawTerms rpy_gain_msg_{};
  spinal_msgs__msg__RollPitchYawTerm rpy_gain_buf_[MAX_RPY_TERMS_SIZE]{};

  spinal_msgs__msg__PMatrixPseudoInverseWithInertia p_matrix_msg_{};
  spinal_msgs__msg__PMatrixPseudoInverseUnit p_matrix_buf_[MAX_P_MATRIX_SIZE]{};

  spinal_msgs__msg__TorqueAllocationMatrixInv torque_allocation_msg_{};
  spinal_msgs__msg__Vector3Int16 torque_allocation_buf_[MAX_TORQUE_ALLOC_SIZE]{};

  spinal_msgs__msg__DesireCoord offset_rot_msg_{};
  spinal_msgs__msg__PositionControlConfig position_config_msg_{};
  spinal_msgs__msg__HealthConfig health_config_msg_{};
  float position_thrust_conversion_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_yaw_acceleration_conversion_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_z_p_gain_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_z_i_gain_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_z_d_gain_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_yaw_p_gain_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_yaw_i_gain_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float position_yaw_d_gain_buf_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  spinal_msgs__msg__PositionControlSetpoint position_setpoint_msg_{};
  std_msgs__msg__Empty network_heartbeat_msg_{};

  std_msgs__msg__UInt8 config_ack_msg_{};
  spinal_msgs__msg__FlightStatus flight_status_msg_{};
  spinal_msgs__msg__RollPitchYawTerms control_term_msg_{};
  spinal_msgs__msg__RollPitchYawTerm control_term_buf_[MAX_RPY_TERMS_SIZE]{};
  spinal_msgs__msg__RollPitchYawTerm control_feedback_state_msg_{};
  spinal_msgs__msg__ApplicationCapabilities application_capabilities_msg_{};
  spinal_msgs__msg__ConfigFlashStatus config_flash_status_msg_{};
  spinal_msgs__msg__FlightParameterTable flight_parameter_table_msg_{};
  spinal_msgs__msg__MotorInfo flight_parameter_pwm_motor_info_buf_[MAX_PWM_MOTOR_INFO_SIZE]{};
  uint32_t last_flight_status_sequence_{ 0U };
  uint32_t last_flight_status_publish_ms_{ 0U };
  uint32_t last_storage_status_publish_ms_{ 0U };

  std_srvs__srv__SetBool_Request att_control_req_{};
  std_srvs__srv__SetBool_Response att_control_res_{};
  std_srvs__srv__SetBool_Request position_control_req_{};
  std_srvs__srv__SetBool_Response position_control_res_{};
  spinal_msgs__srv__ManageFlightParameters_Request parameter_database_req_{};
  spinal_msgs__srv__ManageFlightParameters_Response parameter_database_res_{};
  spinal_msgs__srv__ManageConfigFlash_Request config_flash_req_{};
  spinal_msgs__srv__ManageConfigFlash_Response config_flash_res_{};

  void configure_message_storage_();
  void fillControlTerms_(const FlightControlRpyTerms &src);
  void fillControlFeedback_(const FlightControlRpyTerm &src);
  void fillConfigFlashStatus_(spinal_msgs__msg__ConfigFlashStatus &msg) const;

  void lock_control_();
  void unlock_control_();

  static FlightControlRosModule *instance_;
  static void flightConfigCallbackStatic_(const void *msgin);
  static void uavInfoCallbackStatic_(const void *msgin);
  static void gimbalDofCallbackStatic_(const void *msgin);
  static void fourAxisCommandCallbackStatic_(const void *msgin);
  static void rpyGainCallbackStatic_(const void *msgin);
  static void pMatrixCallbackStatic_(const void *msgin);
  static void torqueAllocationCallbackStatic_(const void *msgin);
  static void offsetRotCallbackStatic_(const void *msgin);
  static void positionConfigCallbackStatic_(const void *msgin);
  static void healthConfigCallbackStatic_(const void *msgin);
  static void positionSetpointCallbackStatic_(const void *msgin);
  static void networkHeartbeatCallbackStatic_(const void *msgin);
  static void attitudeControlCallbackStatic_(const void *req_msg, void *res_msg);
  static void positionControlCallbackStatic_(const void *req_msg, void *res_msg);
  static void parameterDatabaseCallbackStatic_(const void *req_msg, void *res_msg);
  static void configFlashCallbackStatic_(const void *req_msg, void *res_msg);
};

#endif  // !SIMULATION
