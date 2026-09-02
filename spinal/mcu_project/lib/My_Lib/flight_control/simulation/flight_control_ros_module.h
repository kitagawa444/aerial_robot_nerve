#pragma once

#ifdef SIMULATION

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spinal_msgs/msg/application_capabilities.hpp>
#include <spinal_msgs/msg/communication_status.hpp>
#include <spinal_msgs/msg/config_flash_status.hpp>
#include <spinal_msgs/msg/desire_coord.hpp>
#include <spinal_msgs/msg/flight_config_cmd.hpp>
#include <spinal_msgs/msg/flight_parameter_table.hpp>
#include <spinal_msgs/msg/flight_status.hpp>
#include <spinal_msgs/msg/four_axis_command.hpp>
#include <spinal_msgs/msg/health_config.hpp>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.hpp>
#include <spinal_msgs/msg/position_control_config.hpp>
#include <spinal_msgs/msg/position_control_setpoint.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_term.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.hpp>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.hpp>
#include <spinal_msgs/msg/uav_info.hpp>
#include <spinal_msgs/srv/manage_config_flash.hpp>
#include <spinal_msgs/srv/manage_flight_parameters.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "flashmemory/config_flash_database.h"
#include "flashmemory/config_flash_storage_sim.h"
#include "flight_control/flight_control.h"
#include "rc/crsf_input.h"
#include "rc/simulation/crsf_posix_serial_transport.h"
#include "rc/simulation/crsf_ros_adapter.h"
#include "state_estimate/state_estimate.h"
#include "thruster/simulation/thruster_manager.h"

class FlightControlRosModule final {
public:
  FlightControlRosModule() = default;
  ~FlightControlRosModule() = default;

  void init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
            StateEstimate *estimator, ThrusterManager *thruster);

  FlightControl *getFlightControlCore() { return &flight_control_; }

  void activate();
  void deactivate();
  void update();
  void publish();

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  FlightControl flight_control_;
  ConfigFlashDatabase config_flash_database_;
  ConfigFlashStorageSim config_flash_storage_;
  CrsfPosixSerialTransport crsf_transport_;
  CrsfFlightControlSink crsf_control_sink_;
  CrsfInput crsf_input_;
  CrsfRosAdapterSim crsf_ros_adapter_;
  ThrusterManager *thruster_{nullptr};
  StateEstimate *estimator_{nullptr};
  bool initialized_{false};

  rclcpp::Subscription<spinal_msgs::msg::FlightConfigCmd>::SharedPtr
      flight_config_sub_;
  rclcpp::Subscription<spinal_msgs::msg::UavInfo>::SharedPtr uav_info_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gimbal_dof_sub_;
  rclcpp::Subscription<spinal_msgs::msg::FourAxisCommand>::SharedPtr
      four_axis_cmd_sub_;
  rclcpp::Subscription<spinal_msgs::msg::RollPitchYawTerms>::SharedPtr
      rpy_gain_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>::
      SharedPtr p_matrix_sub_;
  rclcpp::Subscription<spinal_msgs::msg::TorqueAllocationMatrixInv>::SharedPtr
      torque_allocation_sub_;
  rclcpp::Subscription<spinal_msgs::msg::DesireCoord>::SharedPtr
      offset_rot_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sim_voltage_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PositionControlConfig>::SharedPtr
      position_config_sub_;
  rclcpp::Subscription<spinal_msgs::msg::HealthConfig>::SharedPtr
      health_config_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PositionControlSetpoint>::SharedPtr
      position_setpoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr network_heartbeat_sub_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      config_ack_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      spinal_msgs::msg::FlightStatus>::SharedPtr flight_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      spinal_msgs::msg::RollPitchYawTerms>::SharedPtr control_term_pub_;
  rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::RollPitchYawTerm>::
      SharedPtr control_feedback_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      std_msgs::msg::Float32MultiArray>::SharedPtr gyro_moment_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
      gimbal_control_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      spinal_msgs::msg::ApplicationCapabilities>::SharedPtr
      application_capabilities_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      spinal_msgs::msg::ConfigFlashStatus>::SharedPtr config_flash_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::FlightParameterTable>::
      SharedPtr flight_parameter_table_pub_;
  rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::CommunicationStatus>::
      SharedPtr communication_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr att_control_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr position_control_srv_;
  rclcpp::Service<spinal_msgs::srv::ManageFlightParameters>::SharedPtr
      parameter_database_srv_;
  rclcpp::Service<spinal_msgs::srv::ManageConfigFlash>::SharedPtr
      config_flash_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_;
  std::mutex control_mutex_;
  uint32_t last_flight_status_sequence_{0U};
  rclcpp::Time last_flight_status_publish_time_{};
  rclcpp::Time last_storage_status_publish_time_{};
  uint32_t last_communication_status_publish_ms_{0U};
  uint32_t previous_rc_valid_frame_count_{0U};

  void configureRosIo_();
  void
  flightConfigCallback_(const spinal_msgs::msg::FlightConfigCmd::SharedPtr msg);
  void uavInfoCallback_(const spinal_msgs::msg::UavInfo::SharedPtr msg);
  void gimbalDofCallback_(const std_msgs::msg::UInt8::SharedPtr msg);
  void fourAxisCommandCallback_(
      const spinal_msgs::msg::FourAxisCommand::SharedPtr msg);
  void
  rpyGainCallback_(const spinal_msgs::msg::RollPitchYawTerms::SharedPtr msg);
  void pMatrixCallback_(
      const spinal_msgs::msg::PMatrixPseudoInverseWithInertia::SharedPtr msg);
  void torqueAllocationCallback_(
      const spinal_msgs::msg::TorqueAllocationMatrixInv::SharedPtr msg);
  void offsetRotCallback_(const spinal_msgs::msg::DesireCoord::SharedPtr msg);
  void simVoltageCallback_(const std_msgs::msg::Float32::SharedPtr msg);
  void positionConfigCallback_(
      const spinal_msgs::msg::PositionControlConfig::SharedPtr msg);
  void
  healthConfigCallback_(const spinal_msgs::msg::HealthConfig::SharedPtr msg);
  void positionSetpointCallback_(
      const spinal_msgs::msg::PositionControlSetpoint::SharedPtr msg);
  void networkHeartbeatCallback_(const std_msgs::msg::Empty::SharedPtr msg);
  void attitudeControlCallback_(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void positionControlCallback_(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void parameterDatabaseCallback_(
      const std::shared_ptr<spinal_msgs::srv::ManageFlightParameters::Request>
          req,
      std::shared_ptr<spinal_msgs::srv::ManageFlightParameters::Response> res);
  void configFlashCallback_(
      const std::shared_ptr<spinal_msgs::srv::ManageConfigFlash::Request> req,
      std::shared_ptr<spinal_msgs::srv::ManageConfigFlash::Response> res);
  void
  rebootCallback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void fillConfigFlashStatus_(spinal_msgs::msg::ConfigFlashStatus &msg) const;
  void publishCommunicationStatus_(const rclcpp::Time &now);
};

#endif // SIMULATION
