#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spinal_msgs/msg/application_capabilities.hpp>
#include <spinal_msgs/msg/communication_status.hpp>
#include <spinal_msgs/msg/config_flash_status.hpp>
#include <spinal_msgs/msg/desire_coord.hpp>
#include <spinal_msgs/msg/external_state_measurement.hpp>
#include <spinal_msgs/msg/flight_config_cmd.hpp>
#include <spinal_msgs/msg/flight_parameter_table.hpp>
#include <spinal_msgs/msg/flight_status.hpp>
#include <spinal_msgs/msg/four_axis_command.hpp>
#include <spinal_msgs/msg/health_config.hpp>
#include <spinal_msgs/msg/imu.hpp>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.hpp>
#include <spinal_msgs/msg/position_control_config.hpp>
#include <spinal_msgs/msg/position_control_setpoint.hpp>
#include <spinal_msgs/msg/pwm_info.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.hpp>
#include <spinal_msgs/msg/state_estimate.hpp>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.hpp>
#include <spinal_msgs/msg/uav_info.hpp>
#include <spinal_msgs/srv/manage_config_flash.hpp>
#include <spinal_msgs/srv/manage_flight_parameters.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "communication/spinal_link_configuration_conversion.h"
#include "communication/spinal_link_protocol.h"
#include "communication_monitor.h"
#include "flashmemory/parameter_database.h"
#include "flight_control/flight_control_ros_adapter.h"
#include "spinal_dds_endpoint.h"

namespace aerial::vehicle {
namespace {
static_assert(sizeof(FlightParameterSnapshot) < 0x10000U,
              "Flight-parameter snapshot exceeds the chunk protocol");
static_assert(sizeof(RebootRequest) == 4U,
              "Reboot request wire layout changed");

template <typename Destination, typename SourceType>
void copyHealthStatus(Destination &destination, const SourceType &source) {
  destination.state = source.state;
  destination.reason_mask = source.reason_mask;
  destination.last_change_ms = source.last_change_ms;
}
} // namespace

class SpinalRos2Bridge : public rclcpp::Node {
public:
  SpinalRos2Bridge()
      : Node("spinal_ros2_bridge"), communication_monitor_(endpoint_) {
    const auto domain_id =
        declare_parameter<int>("spinal_link_dds_domain_id", 0);
    communication_monitor_.setWifiInterface(
        declare_parameter<std::string>("wifi_interface", ""));

    imu_pub_ =
        create_publisher<spinal_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    state_pub_ = create_publisher<spinal_msgs::msg::StateEstimate>(
        "state_estimate", rclcpp::SensorDataQoS());
    flight_status_pub_ = create_publisher<spinal_msgs::msg::FlightStatus>(
        "fc/flight_status", rclcpp::QoS(10));
    battery_pub_ = create_publisher<std_msgs::msg::Float32>(
        "battery_voltage_status", rclcpp::QoS(10));
    config_ack_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "flight_config_ack", rclcpp::QoS(1));
    application_capabilities_pub_ =
        create_publisher<spinal_msgs::msg::ApplicationCapabilities>(
            "fc/application_capabilities", rclcpp::QoS(1));
    config_flash_status_pub_ =
        create_publisher<spinal_msgs::msg::ConfigFlashStatus>(
            "fc/config_flash/status", rclcpp::QoS(1));
    flight_parameter_table_pub_ =
        create_publisher<spinal_msgs::msg::FlightParameterTable>(
            "fc/flight_parameters/table", rclcpp::QoS(1));
    communication_status_pub_ =
        create_publisher<spinal_msgs::msg::CommunicationStatus>(
            "fc/communication_status", rclcpp::QoS(1));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));

    flight_command_sub_ =
        create_subscription<spinal_msgs::msg::FlightConfigCmd>(
            "flight_config_cmd", rclcpp::SystemDefaultsQoS(),
            [this](const spinal_msgs::msg::FlightConfigCmd &message) {
              sendFlightCommand(message);
            });
    external_state_sub_ =
        create_subscription<spinal_msgs::msg::ExternalStateMeasurement>(
            "external_state_measurement", rclcpp::SensorDataQoS(),
            [this](const spinal_msgs::msg::ExternalStateMeasurement &message) {
              sendExternalState(message);
            });
    position_setpoint_sub_ =
        create_subscription<spinal_msgs::msg::PositionControlSetpoint>(
            "position_control/setpoint", rclcpp::SensorDataQoS(),
            [this](const spinal_msgs::msg::PositionControlSetpoint &message) {
              sendPositionSetpoint(message);
            });
    health_config_sub_ = create_subscription<spinal_msgs::msg::HealthConfig>(
        "health/config", rclcpp::SystemDefaultsQoS(),
        [this](const spinal_msgs::msg::HealthConfig &message) {
          sendHealthConfig(message);
        });
    network_heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
        "network/heartbeat", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Empty &) {
          sendHeartbeat(MessageId::NETWORK_HEARTBEAT);
        });
    uav_info_sub_ = create_subscription<spinal_msgs::msg::UavInfo>(
        "uav_info", rclcpp::SystemDefaultsQoS(),
        [this](const spinal_msgs::msg::UavInfo &message) {
          sendAirframeConfig(message);
        });
    gimbal_dof_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "gimbal_dof", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::UInt8 &message) {
          sendConfiguration(ConfigurationKind::GIMBAL_DOF, message.data);
        });
    four_axis_command_sub_ =
        create_subscription<spinal_msgs::msg::FourAxisCommand>(
            "four_axes/command", rclcpp::SystemDefaultsQoS(),
            [this](const spinal_msgs::msg::FourAxisCommand &message) {
              sendFourAxisCommand(message);
            });
    pwm_info_sub_ = create_subscription<spinal_msgs::msg::PwmInfo>(
        "motor_info", rclcpp::SystemDefaultsQoS(),
        [this](const spinal_msgs::msg::PwmInfo &message) {
          sendPwmInfo(message);
        });
    rpy_gain_sub_ = create_subscription<spinal_msgs::msg::RollPitchYawTerms>(
        "rpy/gain", rclcpp::SystemDefaultsQoS(),
        [this](const spinal_msgs::msg::RollPitchYawTerms &message) {
          sendRpyGains(message);
        });
    p_matrix_sub_ =
        create_subscription<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>(
            "p_matrix_pseudo_inverse_inertia", rclcpp::SystemDefaultsQoS(),
            [this](const spinal_msgs::msg::PMatrixPseudoInverseWithInertia
                       &message) { sendPMatrix(message); });
    torque_allocation_sub_ =
        create_subscription<spinal_msgs::msg::TorqueAllocationMatrixInv>(
            "torque_allocation_matrix_inv", rclcpp::SystemDefaultsQoS(),
            [this](const spinal_msgs::msg::TorqueAllocationMatrixInv &message) {
              sendTorqueAllocation(message);
            });
    offset_rotation_sub_ = create_subscription<spinal_msgs::msg::DesireCoord>(
        "desire_coordinate", rclcpp::SystemDefaultsQoS(),
        [this](const spinal_msgs::msg::DesireCoord &message) {
          sendOffsetRotation(message);
        });
    position_config_sub_ =
        create_subscription<spinal_msgs::msg::PositionControlConfig>(
            "position_control/config", rclcpp::SystemDefaultsQoS(),
            [this](const spinal_msgs::msg::PositionControlConfig &message) {
              sendPositionConfig(message);
            });

    attitude_control_service_ = create_service<std_srvs::srv::SetBool>(
        "set_attitude_control",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
          const uint8_t enabled = request->data ? 1U : 0U;
          sendConfiguration(ConfigurationKind::ATTITUDE_CONTROL_ENABLED,
                            enabled);
          response->success = true;
        });
    position_control_service_ = create_service<std_srvs::srv::SetBool>(
        "set_position_control",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
          const uint8_t enabled = request->data ? 1U : 0U;
          sendConfiguration(ConfigurationKind::POSITION_CONTROL_ENABLED,
                            enabled);
          response->success = true;
        });
    config_flash_service_ = create_service<spinal_msgs::srv::ManageConfigFlash>(
        "fc/config_flash",
        [this](
            const spinal_msgs::srv::ManageConfigFlash::Request::SharedPtr
                request,
            spinal_msgs::srv::ManageConfigFlash::Response::SharedPtr response) {
          manageConfigFlash(*request, *response);
        });
    flight_parameter_service_ =
        create_service<spinal_msgs::srv::ManageFlightParameters>(
            "fc/parameters",
            [this](const spinal_msgs::srv::ManageFlightParameters::Request::
                       SharedPtr request,
                   spinal_msgs::srv::ManageFlightParameters::Response::SharedPtr
                       response) {
              manageFlightParameters(*request, *response);
            });
    reboot_service_ = create_service<std_srvs::srv::Trigger>(
        "fc/reboot",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          requestReboot(*response);
        });
    bootloader_service_ = create_service<std_srvs::srv::Trigger>(
        "enter_bootloader",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          requestBootloader(*response);
        });

    endpoint_.setFrameCallback(
        [this](const Frame &frame) { handleSpinalLinkFrame(frame); });
    if (!endpoint_.start("spinal_ros2_bridge",
                         static_cast<uint32_t>(domain_id))) {
      throw std::runtime_error("failed to start the Spinal DDS endpoint");
    }

    ros_heartbeat_timer_ =
        create_wall_timer(std::chrono::milliseconds(250), [this]() {
          sendHeartbeat(MessageId::ROS_HEARTBEAT);
        });
    communication_status_timer_ =
        create_wall_timer(std::chrono::milliseconds(500),
                          [this]() { publishCommunicationStatus(); });
    RCLCPP_INFO(get_logger(),
                "ROS 2 Bridge connected to Spinal Link DDS domain %ld",
                domain_id);
  }

  ~SpinalRos2Bridge() override { endpoint_.stop(); }

private:
  uint64_t timestampMicros() {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000LL);
  }

  static const char *linkStateName(CommunicationLinkState state) {
    switch (state) {
    case CommunicationLinkState::DISCONNECTED:
      return "DISCONNECTED";
    case CommunicationLinkState::CONNECTING:
      return "CONNECTING";
    case CommunicationLinkState::CONNECTED:
      return "CONNECTED";
    case CommunicationLinkState::STALE:
      return "STALE";
    case CommunicationLinkState::DISABLED:
      return "DISABLED";
    case CommunicationLinkState::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
    }
  }

  static uint8_t diagnosticLevel(CommunicationLinkState state) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    switch (state) {
    case CommunicationLinkState::CONNECTED:
    case CommunicationLinkState::DISABLED:
      return DiagnosticStatus::OK;
    case CommunicationLinkState::DISCONNECTED:
    case CommunicationLinkState::ERROR:
      return DiagnosticStatus::ERROR;
    case CommunicationLinkState::UNKNOWN:
      return DiagnosticStatus::STALE;
    default:
      return DiagnosticStatus::WARN;
    }
  }

  static void
  fillLinkMessage(spinal_msgs::msg::CommunicationLinkStatus &message,
                  const CommunicationLinkSnapshot &source) {
    message.name = source.name;
    message.state = static_cast<uint8_t>(source.state);
    message.data_stale = source.data_stale;
    message.interface_up = source.interface_up;
    message.ip_assigned = source.ip_assigned;
    message.transport_matched = source.transport_matched;
    message.last_receive_age_ms = source.last_receive_age_ms;
    message.receive_rate_hz = source.receive_rate_hz;
    message.transmit_rate_hz = source.transmit_rate_hz;
    message.received_count = source.received_count;
    message.transmitted_count = source.transmitted_count;
    message.error_count = source.error_count;
    message.rssi_dbm = source.rssi_dbm;
    message.quality_percent = source.quality_percent;
    message.detail = source.detail;
  }

  static diagnostic_msgs::msg::DiagnosticStatus
  makeLinkDiagnostic(const CommunicationLinkSnapshot &source) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnosticLevel(source.state);
    status.name = "Spinal Link/" + source.name;
    status.hardware_id = "spinal";
    status.message =
        std::string(linkStateName(source.state)) + ": " + source.detail;
    const auto add = [&status](const std::string &key,
                               const std::string &value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };
    add("last_receive_age_ms", std::to_string(source.last_receive_age_ms));
    add("receive_rate_hz", std::to_string(source.receive_rate_hz));
    add("transmit_rate_hz", std::to_string(source.transmit_rate_hz));
    add("received_count", std::to_string(source.received_count));
    add("transmitted_count", std::to_string(source.transmitted_count));
    add("error_count", std::to_string(source.error_count));
    add("quality_percent", std::to_string(source.quality_percent));
    add("rssi_dbm", std::to_string(source.rssi_dbm));
    return status;
  }

  void publishCommunicationStatus() {
    const CommunicationStatusSnapshot snapshot =
        communication_monitor_.sample();
    spinal_msgs::msg::CommunicationStatus message;
    message.header.stamp = now();
    fillLinkMessage(message.micro_usb, snapshot.micro_usb);
    fillLinkMessage(message.ros, snapshot.ros);
    fillLinkMessage(message.wifi, snapshot.wifi);
    fillLinkMessage(message.rc, snapshot.rc);
    message.ack.state = static_cast<uint8_t>(snapshot.ack.state);
    message.ack.pending_count = snapshot.ack.pending_count;
    message.ack.oldest_pending_age_ms = snapshot.ack.oldest_pending_age_ms;
    message.ack.received_count = snapshot.ack.received_count;
    message.ack.retry_count = snapshot.ack.retry_count;
    message.ack.timeout_count = snapshot.ack.timeout_count;
    message.ack.duplicate_count = snapshot.ack.duplicate_count;
    message.ack.rejected_count = snapshot.ack.rejected_count;
    message.ack.last_rtt_ms = snapshot.ack.last_rtt_ms;
    message.ack.last_message_id = snapshot.ack.last_message_id;
    message.ack.last_request_id = snapshot.ack.last_request_id;
    message.ack.detail = snapshot.ack.detail;
    communication_status_pub_->publish(message);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = message.header.stamp;
    diagnostics.status.push_back(makeLinkDiagnostic(snapshot.micro_usb));
    diagnostics.status.push_back(makeLinkDiagnostic(snapshot.ros));
    diagnostics.status.push_back(makeLinkDiagnostic(snapshot.wifi));
    diagnostics.status.push_back(makeLinkDiagnostic(snapshot.rc));
    diagnostic_msgs::msg::DiagnosticStatus ack;
    ack.level = diagnosticLevel(snapshot.ack.state);
    ack.name = "Spinal Link/ACK delivery";
    ack.hardware_id = "spinal";
    ack.message = std::string(linkStateName(snapshot.ack.state)) + ": " +
                  snapshot.ack.detail;
    const auto add_ack = [&ack](const std::string &key,
                                const std::string &value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      ack.values.push_back(pair);
    };
    add_ack("pending_count", std::to_string(snapshot.ack.pending_count));
    add_ack("retry_count", std::to_string(snapshot.ack.retry_count));
    add_ack("timeout_count", std::to_string(snapshot.ack.timeout_count));
    add_ack("duplicate_count", std::to_string(snapshot.ack.duplicate_count));
    add_ack("rejected_count", std::to_string(snapshot.ack.rejected_count));
    add_ack("last_rtt_ms", std::to_string(snapshot.ack.last_rtt_ms));
    add_ack("last_message_id", std::to_string(snapshot.ack.last_message_id));
    add_ack("last_request_id", std::to_string(snapshot.ack.last_request_id));
    diagnostics.status.push_back(ack);
    diagnostics_pub_->publish(diagnostics);
  }

  template <typename Payload>
  void publishFrame(MessageId message_id, const Payload &payload,
                    Reliability reliability) {
    Frame frame;
    setPayload(frame, message_id, payload, ++frame_sequence_, timestampMicros(),
               Source::ROS2, reliability);
    if (!endpoint_.publish(frame))
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Spinal Link DDS write failed");
  }

  void sendHeartbeat(MessageId message_id) {
    Heartbeat heartbeat{};
    heartbeat.uptime_ms = static_cast<uint32_t>(timestampMicros() / 1000U);
    heartbeat.link_state = spinal_msgs::msg::FlightStatus::LINK_CONNECTED;
    publishFrame(message_id, heartbeat, Reliability::BEST_EFFORT);
  }

  void sendFlightCommand(const spinal_msgs::msg::FlightConfigCmd &message) {
    FlightCommand command{};
    command.command_id = ++command_sequence_;
    command.issued_at_ms = static_cast<uint32_t>(timestampMicros() / 1000U);
    command.command = message.cmd;
    command.source = static_cast<uint8_t>(CommandSource::ROS2);
    publishFrame(MessageId::FLIGHT_COMMAND, command, Reliability::RELIABLE);
  }

  void
  sendExternalState(const spinal_msgs::msg::ExternalStateMeasurement &message) {
    ExternalStateMeasurement measurement{};
    measurement.estimate_mode = message.estimate_mode;
    measurement.field_mask = message.field_mask;
    for (size_t axis = 0; axis < 3; ++axis) {
      measurement.position[axis] = message.position[axis];
      measurement.position_variance[axis] = message.position_variance[axis];
      measurement.velocity[axis] = message.velocity[axis];
      measurement.velocity_variance[axis] = message.velocity_variance[axis];
      measurement.attitude_variance[axis] = message.attitude_variance[axis];
      measurement.angular_velocity[axis] = message.angular_velocity[axis];
    }
    for (size_t axis = 0; axis < 4; ++axis)
      measurement.attitude_xyzw[axis] = message.attitude[axis];
    publishFrame(MessageId::EXTERNAL_STATE_MEASUREMENT, measurement,
                 Reliability::BEST_EFFORT);
  }

  void sendPositionSetpoint(
      const spinal_msgs::msg::PositionControlSetpoint &message) {
    PositionControlSetpoint setpoint{};
    for (size_t axis = 0; axis < 3; ++axis) {
      setpoint.position[axis] = message.position[axis];
      setpoint.velocity[axis] = message.velocity[axis];
      setpoint.acceleration[axis] = message.acceleration[axis];
    }
    setpoint.yaw = message.yaw;
    setpoint.yaw_rate = message.yaw_rate;
    setpoint.yaw_acceleration = message.yaw_acceleration;
    setpoint.initial_height = message.initial_height;
    setpoint.horizontal_control_mode = message.horizontal_control_mode;
    setpoint.active = message.active;
    setpoint.manual_control_allowed = message.manual_control_allowed;
    setpoint.landing = message.landing;
    publishFrame(MessageId::POSITION_CONTROL_SETPOINT, setpoint,
                 Reliability::BEST_EFFORT);
  }

  void sendHealthConfig(const spinal_msgs::msg::HealthConfig &message) {
    HealthConfig config{};
    config.battery_cell_count = message.battery_cell_count;
    config.battery_low_percentage = message.battery_low_percentage;
    config.battery_hysteresis_percentage =
        message.battery_hysteresis_percentage;
    config.battery_high_cell_threshold = message.battery_high_cell_threshold;
    config.battery_resistance = message.battery_resistance;
    config.battery_resistance_voltage_rate =
        message.battery_resistance_voltage_rate;
    config.battery_hovering_current = message.battery_hovering_current;
    config.battery_debounce_ms = message.battery_debounce_ms;
    config.primary_imu_timeout_ms = message.primary_imu_timeout_ms;
    config.control_loop_deadline_ms = message.control_loop_deadline_ms;
    config.control_loop_miss_limit = message.control_loop_miss_limit;
    publishFrame(MessageId::HEALTH_CONFIG, config, Reliability::RELIABLE);
  }

  template <typename Configuration>
  void sendConfiguration(ConfigurationKind kind,
                         const Configuration &configuration) {
    static_assert(std::is_trivially_copyable<Configuration>::value,
                  "Spinal Link configuration must be trivially copyable");
    const auto *bytes = reinterpret_cast<const uint8_t *>(&configuration);
    const uint32_t update_id = ++configuration_sequence_;
    for (size_t offset = 0U; offset < sizeof(Configuration);
         offset += kChunkDataSize) {
      ConfigurationChunk chunk{};
      chunk.update_id = update_id;
      chunk.kind = static_cast<uint16_t>(kind);
      chunk.total_size = sizeof(Configuration);
      chunk.offset = static_cast<uint16_t>(offset);
      const size_t remaining = sizeof(Configuration) - offset;
      chunk.chunk_size = static_cast<uint16_t>(
          std::min(remaining, static_cast<size_t>(kChunkDataSize)));
      std::memcpy(chunk.data, bytes + offset, chunk.chunk_size);
      publishFrame(MessageId::CONFIGURATION_CHUNK, chunk,
                   Reliability::RELIABLE);
    }
  }

  void sendAirframeConfig(const spinal_msgs::msg::UavInfo &message) {
    AirframeConfig config{};
    config.motor_count = message.motor_num;
    config.uav_model = static_cast<int8_t>(message.uav_model);
    sendConfiguration(ConfigurationKind::AIRFRAME, config);
  }

  void sendFourAxisCommand(const spinal_msgs::msg::FourAxisCommand &message) {
    FourAxisConfiguration command{};
    for (size_t axis = 0U; axis < 3U; ++axis)
      command.angles[axis] = message.angles[axis];
    command.base_thrust_count = static_cast<uint8_t>(
        std::min(message.base_thrust.size(), kFlightMotorCount));
    for (size_t motor = 0U; motor < command.base_thrust_count; ++motor)
      command.base_thrust[motor] = message.base_thrust[motor];
    sendConfiguration(ConfigurationKind::FOUR_AXIS_COMMAND, command);
  }

  void sendPwmInfo(const spinal_msgs::msg::PwmInfo &message) {
    PwmConfiguration info{};
    info.min_pwm = message.min_pwm;
    info.max_pwm = message.max_pwm;
    info.min_thrust = message.min_thrust;
    info.force_landing_thrust = message.force_landing_thrust;
    info.conversion_mode = message.pwm_conversion_mode;
    info.motor_info_count = static_cast<uint8_t>(
        std::min(message.motor_info.size(), kPwmMotorInfoCount));
    for (size_t motor = 0U; motor < info.motor_info_count; ++motor) {
      info.motor_info[motor].voltage = message.motor_info[motor].voltage;
      info.motor_info[motor].max_thrust = message.motor_info[motor].max_thrust;
      for (size_t coefficient = 0U; coefficient < 5U; ++coefficient)
        info.motor_info[motor].polynomial[coefficient] =
            message.motor_info[motor].polynominal[coefficient];
    }
    sendConfiguration(ConfigurationKind::PWM_INFO, info);
  }

  void sendRpyGains(const spinal_msgs::msg::RollPitchYawTerms &message) {
    RpyGainsConfiguration gains{};
    gains.motor_count = static_cast<uint8_t>(
        std::min(message.motors.size(), kFlightMotorCount));
    for (size_t motor = 0U; motor < gains.motor_count; ++motor) {
      const auto &source = message.motors[motor];
      auto &destination = gains.motors[motor];
      destination.roll_p = source.roll_p;
      destination.roll_i = source.roll_i;
      destination.roll_d = source.roll_d;
      destination.pitch_p = source.pitch_p;
      destination.pitch_i = source.pitch_i;
      destination.pitch_d = source.pitch_d;
      destination.yaw_d = source.yaw_d;
    }
    sendConfiguration(ConfigurationKind::ATTITUDE_GAINS, gains);
  }

  void sendPMatrix(
      const spinal_msgs::msg::PMatrixPseudoInverseWithInertia &message) {
    PMatrixConfiguration matrix{};
    matrix.motor_count = static_cast<uint8_t>(
        std::min(message.pseudo_inverse.size(), kFlightMotorCount));
    for (size_t motor = 0U; motor < matrix.motor_count; ++motor) {
      matrix.pseudo_inverse[motor].r = message.pseudo_inverse[motor].r;
      matrix.pseudo_inverse[motor].p = message.pseudo_inverse[motor].p;
      matrix.pseudo_inverse[motor].y = message.pseudo_inverse[motor].y;
    }
    for (size_t element = 0U; element < 6U; ++element)
      matrix.inertia[element] = message.inertia[element];
    sendConfiguration(ConfigurationKind::P_MATRIX, matrix);
  }

  void sendTorqueAllocation(
      const spinal_msgs::msg::TorqueAllocationMatrixInv &message) {
    TorqueAllocationConfiguration matrix{};
    matrix.row_count =
        static_cast<uint8_t>(std::min(message.rows.size(), kFlightMotorCount));
    for (size_t motor = 0U; motor < matrix.row_count; ++motor) {
      matrix.rows[motor].x = message.rows[motor].x;
      matrix.rows[motor].y = message.rows[motor].y;
      matrix.rows[motor].z = message.rows[motor].z;
    }
    sendConfiguration(ConfigurationKind::TORQUE_ALLOCATION, matrix);
  }

  void sendOffsetRotation(const spinal_msgs::msg::DesireCoord &message) {
    OffsetRotationConfiguration coordinate{};
    coordinate.roll = message.roll;
    coordinate.pitch = message.pitch;
    coordinate.yaw = message.yaw;
    sendConfiguration(ConfigurationKind::OFFSET_ROTATION, coordinate);
  }

  void
  sendPositionConfig(const spinal_msgs::msg::PositionControlConfig &message) {
    PositionControlConfiguration config{};
    for (size_t axis = 0U; axis < 3U; ++axis) {
      config.position_p[axis] = message.position_p[axis];
      config.position_i[axis] = message.position_i[axis];
      config.velocity_d[axis] = message.velocity_d[axis];
      config.limit_sum[axis] = message.limit_sum[axis];
      config.limit_p[axis] = message.limit_p[axis];
      config.limit_i[axis] = message.limit_i[axis];
      config.limit_d[axis] = message.limit_d[axis];
      config.limit_err_p[axis] = message.limit_err_p[axis];
      config.limit_err_d[axis] = message.limit_err_d[axis];
      config.integral_limit[axis] = message.integral_limit[axis];
    }
    config.yaw_p = message.yaw_p;
    config.yaw_i = message.yaw_i;
    config.yaw_limit_sum = message.yaw_limit_sum;
    config.yaw_limit_err_p = message.yaw_limit_err_p;
    config.yaw_limit_err_i = message.yaw_limit_err_i;
    config.yaw_limit_err_d = message.yaw_limit_err_d;
    config.max_horizontal_acceleration = message.max_horizontal_acceleration;
    config.max_vertical_acceleration = message.max_vertical_acceleration;
    config.max_tilt_angle = message.max_tilt_angle;
    config.motor_count = static_cast<uint8_t>(std::min(
        message.vertical_acceleration_to_thrust.size(), kFlightMotorCount));
    for (size_t motor = 0U; motor < config.motor_count; ++motor) {
      config.vertical_acceleration_to_thrust[motor] =
          message.vertical_acceleration_to_thrust[motor];
      if (motor < message.yaw_acceleration_to_thrust.size())
        config.yaw_acceleration_to_thrust[motor] =
            message.yaw_acceleration_to_thrust[motor];
      if (motor < message.z_p_gain.size())
        config.z_p_gain[motor] = message.z_p_gain[motor];
      if (motor < message.z_i_gain.size())
        config.z_i_gain[motor] = message.z_i_gain[motor];
      if (motor < message.z_d_gain.size())
        config.z_d_gain[motor] = message.z_d_gain[motor];
      if (motor < message.yaw_p_gain.size())
        config.yaw_p_gain[motor] = message.yaw_p_gain[motor];
      if (motor < message.yaw_i_gain.size())
        config.yaw_i_gain[motor] = message.yaw_i_gain[motor];
      if (motor < message.yaw_d_gain.size())
        config.yaw_d_gain[motor] = message.yaw_d_gain[motor];
    }
    config.use_lqi_gains = message.use_lqi_gains ? 1U : 0U;
    config.yaw_rate_feedback_on_spinal =
        message.yaw_rate_feedback_on_spinal ? 1U : 0U;
    config.start_roll_pitch_integration_height =
        message.start_roll_pitch_integration_height;
    config.landing_err_z = message.landing_err_z;
    config.safe_landing_height = message.safe_landing_height;
    config.setpoint_timeout_ms = message.setpoint_timeout_ms;
    config.rc_max_horizontal_velocity = message.rc_max_horizontal_velocity;
    config.rc_max_vertical_velocity = message.rc_max_vertical_velocity;
    config.rc_max_yaw_rate = message.rc_max_yaw_rate;
    config.rc_deadzone = message.rc_deadzone;
    config.rc_timeout_ms = message.rc_timeout_ms;
    config.supervisor.takeoff_height = message.takeoff_height;
    config.supervisor.takeoff_position_tolerance =
        message.takeoff_position_tolerance;
    config.supervisor.takeoff_velocity_tolerance =
        message.takeoff_velocity_tolerance;
    config.supervisor.takeoff_stable_time_ms = message.takeoff_stable_time_ms;
    config.supervisor.landing_speed = message.landing_speed;
    config.supervisor.landed_height = message.landed_height;
    config.supervisor.landed_velocity = message.landed_velocity;
    config.supervisor.landed_stable_time_ms = message.landed_stable_time_ms;
    config.supervisor.rc_authority_timeout_ms = message.rc_authority_timeout_ms;
    sendConfiguration(ConfigurationKind::POSITION_CONTROL, config);
  }

  static void
  fillConfigFlashStatus(spinal_msgs::msg::ConfigFlashStatus &destination,
                        const ConfigFlashStatus &source) {
    destination.valid = source.valid != 0U;
    destination.dirty = source.dirty != 0U;
    destination.reboot_required = source.reboot_required != 0U;
    destination.persistent_storage = source.persistent_storage != 0U;
    destination.active_slot = source.active_slot;
    destination.active_uart3_driver = source.active.uart3_driver;
    destination.pending_uart3_driver = source.pending.uart3_driver;
    destination.active_imu_driver = source.active.imu_driver;
    destination.pending_imu_driver = source.pending.imu_driver;
    destination.active_barometer_enabled =
        source.active.barometer_enabled != 0U;
    destination.pending_barometer_enabled =
        source.pending.barometer_enabled != 0U;
    destination.active_servo_driver = source.active.servo_driver;
    destination.pending_servo_driver = source.pending.servo_driver;
    destination.active_attitude_estimation_enabled =
        source.active.attitude_estimation_enabled != 0U;
    destination.pending_attitude_estimation_enabled =
        source.pending.attitude_estimation_enabled != 0U;
    destination.active_height_estimation_enabled =
        source.active.height_estimation_enabled != 0U;
    destination.pending_height_estimation_enabled =
        source.pending.height_estimation_enabled != 0U;
    destination.active_position_estimation_enabled =
        source.active.position_estimation_enabled != 0U;
    destination.pending_position_estimation_enabled =
        source.pending.position_estimation_enabled != 0U;
    destination.active_flight_control_enabled =
        source.active.flight_control_enabled != 0U;
    destination.pending_flight_control_enabled =
        source.pending.flight_control_enabled != 0U;
    destination.active_motor_output_driver = source.active.motor_output_driver;
    destination.pending_motor_output_driver =
        source.pending.motor_output_driver;
    destination.schema_version = source.schema_version;
    destination.generation = source.generation;
    destination.crc32 = source.crc32;
  }

  void
  manageConfigFlash(const spinal_msgs::srv::ManageConfigFlash::Request &request,
                    spinal_msgs::srv::ManageConfigFlash::Response &response) {
    ConfigFlashRequest transport_request{};
    transport_request.request_id = ++request_sequence_;
    transport_request.command = request.command;
    transport_request.selection.uart3_driver = request.uart3_driver;
    transport_request.selection.imu_driver = request.imu_driver;
    transport_request.selection.barometer_enabled =
        request.barometer_enabled ? 1U : 0U;
    transport_request.selection.servo_driver = request.servo_driver;
    transport_request.selection.attitude_estimation_enabled =
        request.attitude_estimation_enabled ? 1U : 0U;
    transport_request.selection.height_estimation_enabled =
        request.height_estimation_enabled ? 1U : 0U;
    transport_request.selection.position_estimation_enabled =
        request.position_estimation_enabled ? 1U : 0U;
    transport_request.selection.flight_control_enabled =
        request.flight_control_enabled ? 1U : 0U;
    transport_request.selection.motor_output_driver =
        request.motor_output_driver;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      config_response_id_ = 0U;
    }
    publishFrame(MessageId::CONFIG_FLASH_REQUEST, transport_request,
                 Reliability::RELIABLE);
    std::unique_lock<std::mutex> lock(response_mutex_);
    if (!response_condition_.wait_for(lock, std::chrono::seconds(2), [&]() {
          return config_response_id_ == transport_request.request_id;
        })) {
      response.success = false;
      response.result =
          spinal_msgs::srv::ManageConfigFlash::Response::RESULT_STORAGE_ERROR;
      return;
    }
    response.success = config_response_.success != 0U;
    response.result = config_response_.result;
    fillConfigFlashStatus(response.status, config_response_);
  }

  void manageFlightParameters(
      const spinal_msgs::srv::ManageFlightParameters::Request &request,
      spinal_msgs::srv::ManageFlightParameters::Response &response) {
    ParameterRequest transport_request{};
    transport_request.request_id = ++request_sequence_;
    transport_request.command = request.command;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      parameter_response_id_ = 0U;
    }
    publishFrame(MessageId::PARAMETER_REQUEST, transport_request,
                 Reliability::RELIABLE);
    std::unique_lock<std::mutex> lock(response_mutex_);
    if (!response_condition_.wait_for(lock, std::chrono::seconds(3), [&]() {
          return parameter_response_id_ == transport_request.request_id;
        })) {
      response.success = false;
      response.result = spinal_msgs::srv::ManageFlightParameters::Response::
          RESULT_STORAGE_ERROR;
      return;
    }
    response.success = parameter_response_.success != 0U;
    response.result = parameter_response_.result;
    response.valid = parameter_response_.valid != 0U;
    response.dirty = parameter_response_.dirty != 0U;
    response.applied = parameter_response_.applied != 0U;
    response.persistent_storage = parameter_response_.persistent_storage != 0U;
    response.schema_version = parameter_response_.schema_version;
    response.generation = parameter_response_.generation;
    response.crc32 = parameter_response_.crc32;
    response.valid_fields = parameter_response_.valid_fields;
  }

  void requestReboot(std_srvs::srv::Trigger::Response &response) {
    aerial::vehicle::RebootRequest request{};
    request.request_id = ++request_sequence_;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      reboot_response_id_ = 0U;
    }
    publishFrame(MessageId::REBOOT_REQUEST, request, Reliability::RELIABLE);
    std::unique_lock<std::mutex> lock(response_mutex_);
    if (!response_condition_.wait_for(lock, std::chrono::seconds(2), [&]() {
          return reboot_response_id_ == request.request_id;
        })) {
      response.success = false;
      response.message = "FC reboot response timed out";
      return;
    }
    response.success = reboot_response_.success != 0U;
    response.message = response.success ? "FC reboot accepted"
                                        : "FC reboot rejected while armed";
  }

  void requestBootloader(std_srvs::srv::Trigger::Response &response) {
    aerial::vehicle::RebootRequest request{};
    request.request_id = ++request_sequence_;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      bootloader_response_id_ = 0U;
    }
    publishFrame(MessageId::BOOTLOADER_REQUEST, request, Reliability::RELIABLE);
    std::unique_lock<std::mutex> lock(response_mutex_);
    if (!response_condition_.wait_for(lock, std::chrono::seconds(2), [&]() {
          return bootloader_response_id_ == request.request_id;
        })) {
      response.success = false;
      response.message = "FC bootloader response timed out";
      return;
    }
    response.success = bootloader_response_.success != 0U;
    response.message =
        response.success
            ? "motor outputs stopped; ROM bootloader reset accepted"
            : "bootloader reset rejected while armed";
  }

  void handleSpinalLinkFrame(const Frame &frame) {
    switch (static_cast<MessageId>(frame.header.message_id)) {
    case MessageId::IMU:
      publishImu(frame);
      break;
    case MessageId::STATE_ESTIMATE:
      publishStateEstimate(frame);
      break;
    case MessageId::FLIGHT_STATUS:
      publishFlightStatus(frame);
      break;
    case MessageId::BATTERY_STATUS:
      publishBattery(frame);
      break;
    case MessageId::RC_STATUS:
      noteRcStatus(frame);
      break;
    case MessageId::APPLICATION_CAPABILITIES:
      publishApplicationCapabilities(frame);
      break;
    case MessageId::CONFIG_FLASH_STATUS:
      publishConfigFlashStatus(frame);
      break;
    case MessageId::FLIGHT_PARAMETER_STATUS:
      handleFlightParameterStatus(frame);
      break;
    case MessageId::FLIGHT_PARAMETER_CHUNK:
      handleFlightParameterChunk(frame);
      break;
    case MessageId::REBOOT_STATUS:
      handleRebootStatus(frame);
      break;
    case MessageId::BOOTLOADER_STATUS:
      handleBootloaderStatus(frame);
      break;
    case MessageId::CONFIG_ACK:
      publishConfigAck(frame);
      break;
    default:
      break;
    }
  }

  void publishImu(const Frame &frame) {
    Imu payload{};
    if (!getPayload(frame, MessageId::IMU, payload))
      return;
    spinal_msgs::msg::Imu message;
    message.stamp = now();
    for (size_t axis = 0; axis < 3; ++axis) {
      message.acc[axis] = payload.acceleration[axis];
      message.gyro[axis] = payload.angular_velocity[axis];
      message.mag[axis] = payload.magnetic_field[axis];
    }
    for (size_t axis = 0; axis < 4; ++axis)
      message.quaternion[axis] = payload.attitude_xyzw[axis];
    imu_pub_->publish(message);
  }

  void publishStateEstimate(const Frame &frame) {
    StateEstimate payload{};
    if (!getPayload(frame, MessageId::STATE_ESTIMATE, payload))
      return;
    spinal_msgs::msg::StateEstimate message;
    message.stamp = now();
    message.validity = payload.validity;
    for (size_t axis = 0; axis < 3; ++axis) {
      message.position[axis] = payload.position[axis];
      message.velocity[axis] = payload.velocity[axis];
      message.acceleration[axis] = payload.acceleration[axis];
      message.angular_velocity[axis] = payload.angular_velocity[axis];
      message.accelerometer_bias[axis] = payload.accelerometer_bias[axis];
      message.gyroscope_bias[axis] = payload.gyroscope_bias[axis];
    }
    for (size_t axis = 0; axis < 4; ++axis)
      message.attitude[axis] = payload.attitude_xyzw[axis];
    for (size_t index = 0; index < 15; ++index)
      message.covariance_diagonal[index] = payload.covariance_diagonal[index];
    message.rejected_measurements = payload.rejected_measurements;
    state_pub_->publish(message);
  }

  void publishFlightStatus(const Frame &frame) {
    FlightStatus payload{};
    if (!getPayload(frame, MessageId::FLIGHT_STATUS, payload))
      return;
    communication_monitor_.noteFlightStatus(payload.ros_link_state,
                                            payload.network_link_state,
                                            payload.rc_link_state);
    spinal_msgs::msg::FlightStatus message;
    message.stamp = now();
    message.sequence = payload.sequence;
    message.command_sequence = payload.command_sequence;
    message.arming_state = payload.arming_state;
    message.flight_phase = payload.flight_phase;
    message.control_mode = payload.control_mode;
    message.user_intention = payload.user_intention;
    message.authority = payload.authority;
    message.failsafe = payload.failsafe;
    message.last_command = payload.last_command;
    message.last_command_source = payload.last_command_source;
    message.last_command_result = payload.last_command_result;
    message.transition_reason = payload.transition_reason;
    message.ready_to_arm = payload.ready_to_arm != 0U;
    message.attitude_ready = payload.attitude_ready != 0U;
    message.position_ready = payload.position_ready != 0U;
    message.rc_connected = payload.rc_connected != 0U;
    message.ros_link_state = payload.ros_link_state;
    message.network_link_state = payload.network_link_state;
    message.rc_link_state = payload.rc_link_state;
    copyHealthStatus(message.health.overall, payload.health.overall);
    copyHealthStatus(message.health.flight, payload.health.flight);
    copyHealthStatus(message.health.link.summary, payload.health.link);
    message.health.link.ros = payload.health.ros_link_state;
    message.health.link.network = payload.health.network_link_state;
    message.health.link.rc = payload.health.rc_link_state;
    copyHealthStatus(message.health.power, payload.health.power);
    copyHealthStatus(message.health.sensor, payload.health.sensor);
    copyHealthStatus(message.health.estimation, payload.health.estimation);
    copyHealthStatus(message.health.actuator, payload.health.actuator);
    copyHealthStatus(message.health.compute, payload.health.compute);
    copyHealthStatus(message.health.config, payload.health.config);
    message.health.requested_action = payload.health.requested_action;
    message.health.action_source = payload.health.action_source;
    message.health.action_reason_mask = payload.health.action_reason_mask;
    message.health.action_since_ms = payload.health.action_since_ms;
    message.health.battery_voltage = payload.health.battery_voltage;
    message.health.battery_compensated_voltage =
        payload.health.battery_compensated_voltage;
    message.health.battery_percentage = payload.health.battery_percentage;
    message.health.estimation_validity = payload.health.estimation_validity;
    message.health.rejected_measurements = payload.health.rejected_measurements;
    message.takeoff_reference_height = payload.takeoff_reference_height;
    message.takeoff_target_height = payload.takeoff_target_height;
    flight_status_pub_->publish(message);
  }

  void noteRcStatus(const Frame &frame) {
    RcStatus payload{};
    if (!getPayload(frame, MessageId::RC_STATUS, payload))
      return;
    communication_monitor_.noteRcStatus(payload.connected != 0U,
                                        payload.link_quality, payload.rssi_dbm);
  }

  void publishBattery(const Frame &frame) {
    BatteryStatus payload{};
    if (!getPayload(frame, MessageId::BATTERY_STATUS, payload))
      return;
    std_msgs::msg::Float32 message;
    message.data = payload.voltage;
    battery_pub_->publish(message);
  }

  void publishApplicationCapabilities(const Frame &frame) {
    ApplicationCapabilities payload{};
    if (!getPayload(frame, MessageId::APPLICATION_CAPABILITIES, payload))
      return;
    spinal_msgs::msg::ApplicationCapabilities message;
    message.board = payload.board;
    message.capability_mask = payload.capability_mask;
    message.max_motor_count = payload.max_motor_count;
    message.config_schema_version = payload.config_schema_version;
    message.flight_parameter_schema_version =
        payload.flight_parameter_schema_version;
    application_capabilities_pub_->publish(message);
  }

  void publishConfigFlashStatus(const Frame &frame) {
    ConfigFlashStatus payload{};
    if (!getPayload(frame, MessageId::CONFIG_FLASH_STATUS, payload))
      return;
    spinal_msgs::msg::ConfigFlashStatus message;
    fillConfigFlashStatus(message, payload);
    config_flash_status_pub_->publish(message);
    if (payload.request_id != 0U) {
      {
        std::lock_guard<std::mutex> lock(response_mutex_);
        config_response_ = payload;
        config_response_id_ = payload.request_id;
      }
      response_condition_.notify_all();
    }
  }

  void handleFlightParameterStatus(const Frame &frame) {
    FlightParameterStatus payload{};
    if (!getPayload(frame, MessageId::FLIGHT_PARAMETER_STATUS, payload))
      return;
    {
      std::lock_guard<std::mutex> lock(parameter_table_mutex_);
      parameter_table_status_ = payload;
      parameter_table_request_id_ = payload.request_id;
      parameter_table_received_ = 0U;
    }
    if (payload.request_id != 0U) {
      {
        std::lock_guard<std::mutex> lock(response_mutex_);
        parameter_response_ = payload;
        parameter_response_id_ = payload.request_id;
      }
      response_condition_.notify_all();
    }
  }

  void handleFlightParameterChunk(const Frame &frame) {
    FlightParameterChunk chunk{};
    if (!getPayload(frame, MessageId::FLIGHT_PARAMETER_CHUNK, chunk) ||
        chunk.total_size != sizeof(FlightParameterSnapshot) ||
        chunk.chunk_size > kChunkDataSize ||
        static_cast<size_t>(chunk.offset) + chunk.chunk_size >
            sizeof(FlightParameterSnapshot)) {
      return;
    }
    spinal_msgs::msg::FlightParameterTable message;
    bool complete = false;
    {
      std::lock_guard<std::mutex> lock(parameter_table_mutex_);
      if (chunk.request_id != parameter_table_request_id_ ||
          chunk.offset != parameter_table_received_)
        return;
      std::memcpy(parameter_table_bytes_ + chunk.offset, chunk.data,
                  chunk.chunk_size);
      parameter_table_received_ += chunk.chunk_size;
      if (parameter_table_received_ == sizeof(FlightParameterSnapshot)) {
        FlightParameterSnapshot snapshot{};
        std::memcpy(&snapshot, parameter_table_bytes_, sizeof(snapshot));
        const FlightParameterPayload parameters =
            conversion::toInternal(snapshot);
        flight_control_ros::fillFlightParameterTable(
            message, parameters, parameter_table_status_.valid != 0U,
            parameter_table_status_.dirty != 0U,
            parameter_table_status_.applied != 0U,
            parameter_table_status_.persistent_storage != 0U,
            parameter_table_status_.schema_version,
            parameter_table_status_.generation, parameter_table_status_.crc32,
            parameter_table_status_.valid_fields);
        complete = true;
      }
    }
    if (complete)
      flight_parameter_table_pub_->publish(message);
  }

  void handleRebootStatus(const Frame &frame) {
    RebootStatus payload{};
    if (!getPayload(frame, MessageId::REBOOT_STATUS, payload))
      return;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      reboot_response_ = payload;
      reboot_response_id_ = payload.request_id;
    }
    response_condition_.notify_all();
  }

  void handleBootloaderStatus(const Frame &frame) {
    RebootStatus payload{};
    if (!getPayload(frame, MessageId::BOOTLOADER_STATUS, payload))
      return;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      bootloader_response_ = payload;
      bootloader_response_id_ = payload.request_id;
    }
    response_condition_.notify_all();
  }

  void publishConfigAck(const Frame &frame) {
    ConfigAck payload{};
    if (!getPayload(frame, MessageId::CONFIG_ACK, payload))
      return;
    std_msgs::msg::UInt8 message;
    message.data = payload.command;
    config_ack_pub_->publish(message);
  }

  SpinalDdsEndpoint endpoint_;
  CommunicationMonitor communication_monitor_;
  std::atomic<uint32_t> frame_sequence_{0U};
  std::atomic<uint32_t> command_sequence_{0U};
  std::atomic<uint32_t> configuration_sequence_{0U};
  std::atomic<uint32_t> request_sequence_{0U};
  std::mutex response_mutex_;
  std::condition_variable response_condition_;
  uint32_t config_response_id_{0U};
  uint32_t parameter_response_id_{0U};
  uint32_t reboot_response_id_{0U};
  uint32_t bootloader_response_id_{0U};
  ConfigFlashStatus config_response_{};
  FlightParameterStatus parameter_response_{};
  RebootStatus reboot_response_{};
  RebootStatus bootloader_response_{};
  std::mutex parameter_table_mutex_;
  FlightParameterStatus parameter_table_status_{};
  uint32_t parameter_table_request_id_{0U};
  size_t parameter_table_received_{0U};
  uint8_t parameter_table_bytes_[sizeof(FlightParameterSnapshot)]{};
  rclcpp::Publisher<spinal_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<spinal_msgs::msg::StateEstimate>::SharedPtr state_pub_;
  rclcpp::Publisher<spinal_msgs::msg::FlightStatus>::SharedPtr
      flight_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr config_ack_pub_;
  rclcpp::Publisher<spinal_msgs::msg::ApplicationCapabilities>::SharedPtr
      application_capabilities_pub_;
  rclcpp::Publisher<spinal_msgs::msg::ConfigFlashStatus>::SharedPtr
      config_flash_status_pub_;
  rclcpp::Publisher<spinal_msgs::msg::FlightParameterTable>::SharedPtr
      flight_parameter_table_pub_;
  rclcpp::Publisher<spinal_msgs::msg::CommunicationStatus>::SharedPtr
      communication_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
  rclcpp::Subscription<spinal_msgs::msg::FlightConfigCmd>::SharedPtr
      flight_command_sub_;
  rclcpp::Subscription<spinal_msgs::msg::ExternalStateMeasurement>::SharedPtr
      external_state_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PositionControlSetpoint>::SharedPtr
      position_setpoint_sub_;
  rclcpp::Subscription<spinal_msgs::msg::HealthConfig>::SharedPtr
      health_config_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr network_heartbeat_sub_;
  rclcpp::Subscription<spinal_msgs::msg::UavInfo>::SharedPtr uav_info_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gimbal_dof_sub_;
  rclcpp::Subscription<spinal_msgs::msg::FourAxisCommand>::SharedPtr
      four_axis_command_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PwmInfo>::SharedPtr pwm_info_sub_;
  rclcpp::Subscription<spinal_msgs::msg::RollPitchYawTerms>::SharedPtr
      rpy_gain_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>::
      SharedPtr p_matrix_sub_;
  rclcpp::Subscription<spinal_msgs::msg::TorqueAllocationMatrixInv>::SharedPtr
      torque_allocation_sub_;
  rclcpp::Subscription<spinal_msgs::msg::DesireCoord>::SharedPtr
      offset_rotation_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PositionControlConfig>::SharedPtr
      position_config_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr attitude_control_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr position_control_service_;
  rclcpp::Service<spinal_msgs::srv::ManageConfigFlash>::SharedPtr
      config_flash_service_;
  rclcpp::Service<spinal_msgs::srv::ManageFlightParameters>::SharedPtr
      flight_parameter_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bootloader_service_;
  rclcpp::TimerBase::SharedPtr ros_heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr communication_status_timer_;
};
} // namespace aerial::vehicle

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aerial::vehicle::SpinalRos2Bridge>());
  rclcpp::shutdown();
  return 0;
}
