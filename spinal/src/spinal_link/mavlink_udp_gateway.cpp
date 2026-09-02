#include "mavlink_udp_gateway.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace aerial::vehicle {
namespace {
constexpr uint8_t kFlightCommandArm = 0U;
constexpr uint8_t kFlightCommandDisarm = 1U;
constexpr uint8_t kFlightCommandForceLand = 2U;
constexpr uint8_t kFlightCommandTakeoff = 3U;
constexpr uint8_t kFlightCommandLand = 4U;
constexpr uint8_t kFlightCommandHalt = 5U;
constexpr uint16_t kMavCmdForceLand = 31010U;
constexpr float kPi = 3.14159265358979323846F;

uint8_t mavState(const FlightStatus &status) {
  if (status.failsafe == 2U || status.control_mode == 6U)
    return MAV_STATE_EMERGENCY;
  if (status.failsafe != 0U)
    return MAV_STATE_CRITICAL;
  if (status.arming_state != 0U)
    return MAV_STATE_ACTIVE;
  return status.ready_to_arm != 0U ? MAV_STATE_STANDBY : MAV_STATE_CALIBRATING;
}

uint8_t mavResult(uint8_t result) {
  switch (result) {
  case 0U:
  case 2U:
    return MAV_RESULT_ACCEPTED;
  case 1U:
    return MAV_RESULT_UNSUPPORTED;
  case 4U:
  case 7U:
  case 8U:
    return MAV_RESULT_DENIED;
  default:
    return MAV_RESULT_FAILED;
  }
}
} // namespace

struct MavlinkUdpGateway::sockaddr_in_storage {
  sockaddr_in value{};
};

MavlinkUdpGateway::MavlinkUdpGateway(SpinalDdsEndpoint &dds_endpoint)
    : dds_endpoint_(dds_endpoint), peer_(new sockaddr_in_storage) {}

MavlinkUdpGateway::~MavlinkUdpGateway() { stop(); }

bool MavlinkUdpGateway::start(const Config &config) {
  stop();
  config_ = config;
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
    return false;

  int enabled = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
  setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(config_.bind_port);
  if (inet_pton(AF_INET, config_.bind_address.c_str(), &local.sin_addr) != 1 ||
      bind(socket_fd_, reinterpret_cast<const sockaddr *>(&local),
           sizeof(local)) != 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  peer_->value = sockaddr_in{};
  peer_->value.sin_family = AF_INET;
  peer_->value.sin_port = htons(config_.remote_port);
  if (inet_pton(AF_INET, config_.remote_address.c_str(),
                &peer_->value.sin_addr) != 1) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  running_.store(true);
  receive_thread_ = std::thread(&MavlinkUdpGateway::receiveLoop, this);
  return true;
}

void MavlinkUdpGateway::stop() {
  running_.store(false);
  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
    close(socket_fd_);
    socket_fd_ = -1;
  }
  if (receive_thread_.joinable())
    receive_thread_.join();
}

bool MavlinkUdpGateway::running() const { return running_.load(); }

void MavlinkUdpGateway::handleSpinalLinkFrame(const Frame &frame) {
  mavlink_message_t message{};
  switch (static_cast<MessageId>(frame.header.message_id)) {
  case MessageId::HEARTBEAT: {
    Heartbeat heartbeat;
    if (!getPayload(frame, MessageId::HEARTBEAT, heartbeat))
      return;
    mavlink_msg_heartbeat_pack(config_.system_id, config_.component_id,
                               &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, 0U, 0U, MAV_STATE_UNINIT);
    sendMessage(message);
    break;
  }
  case MessageId::FLIGHT_STATUS: {
    FlightStatus status;
    if (!getPayload(frame, MessageId::FLIGHT_STATUS, status))
      return;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (status.arming_state != 0U)
      base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    if (status.control_mode == 2U || status.control_mode == 3U ||
        status.control_mode == 4U)
      base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    mavlink_msg_heartbeat_pack(config_.system_id, config_.component_id,
                               &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, base_mode,
                               status.control_mode, mavState(status));
    sendMessage(message);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_mavlink_command_ != 0U &&
          status.command_sequence != last_command_status_sequence_ &&
          status.last_command == pending_flight_command_) {
        sendCommandAck(pending_mavlink_command_,
                       mavResult(status.last_command_result));
        pending_mavlink_command_ = 0U;
        pending_flight_command_ = 0U;
      }
      last_command_status_sequence_ = status.command_sequence;
    }
    break;
  }
  case MessageId::STATE_ESTIMATE: {
    StateEstimate state;
    if (!getPayload(frame, MessageId::STATE_ESTIMATE, state))
      return;
    mavlink_msg_local_position_ned_pack(
        config_.system_id, config_.component_id, &message,
        static_cast<uint32_t>(frame.header.timestamp_us / 1000U),
        state.position[1], state.position[0], -state.position[2],
        state.velocity[1], state.velocity[0], -state.velocity[2]);
    sendMessage(message);
    break;
  }
  case MessageId::IMU: {
    Imu imu;
    if (!getPayload(frame, MessageId::IMU, imu))
      return;
    constexpr uint16_t fields =
        HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_YACC |
        HIGHRES_IMU_UPDATED_ZACC | HIGHRES_IMU_UPDATED_XGYRO |
        HIGHRES_IMU_UPDATED_YGYRO | HIGHRES_IMU_UPDATED_ZGYRO |
        HIGHRES_IMU_UPDATED_XMAG | HIGHRES_IMU_UPDATED_YMAG |
        HIGHRES_IMU_UPDATED_ZMAG;
    mavlink_msg_highres_imu_pack(
        config_.system_id, config_.component_id, &message,
        frame.header.timestamp_us, imu.acceleration[0], imu.acceleration[1],
        imu.acceleration[2], imu.angular_velocity[0], imu.angular_velocity[1],
        imu.angular_velocity[2], imu.magnetic_field[0], imu.magnetic_field[1],
        imu.magnetic_field[2], 0.0F, 0.0F, 0.0F, 0.0F, fields, 0U);
    sendMessage(message);
    break;
  }
  case MessageId::BATTERY_STATUS: {
    BatteryStatus battery;
    if (!getPayload(frame, MessageId::BATTERY_STATUS, battery))
      return;
    std::array<uint16_t, 10> voltages{};
    voltages.fill(UINT16_MAX);
    voltages[0] =
        static_cast<uint16_t>(std::max(0.0F, battery.voltage) * 1000.0F);
    const int16_t current = std::isfinite(battery.current)
                                ? static_cast<int16_t>(battery.current * 100.0F)
                                : -1;
    const int8_t remaining =
        std::isfinite(battery.percentage)
            ? static_cast<int8_t>(
                  std::max(0.0F, std::min(100.0F, battery.percentage)))
            : -1;
    mavlink_msg_battery_status_pack(
        config_.system_id, config_.component_id, &message, 0U,
        MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX,
        voltages.data(), current, -1, -1, remaining, 0,
        MAV_BATTERY_CHARGE_STATE_UNDEFINED, voltages.data(), 0U, 0U);
    sendMessage(message);
    break;
  }
  case MessageId::EVENT: {
    Event event;
    if (!getPayload(frame, MessageId::EVENT, event))
      return;
    char text[50]{};
    std::snprintf(text, sizeof(text), "FC event=%u reason=0x%08x",
                  event.event_id, event.argument);
    mavlink_msg_statustext_pack(config_.system_id, config_.component_id,
                                &message, event.severity, text,
                                static_cast<uint16_t>(event.sequence), 0U);
    sendMessage(message);
    break;
  }
  default:
    break;
  }
}

void MavlinkUdpGateway::receiveLoop() {
  std::array<uint8_t, 2048> buffer{};
  while (running_.load()) {
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const ssize_t size =
        recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                 reinterpret_cast<sockaddr *>(&source), &source_length);
    if (size <= 0) {
      if (running_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    bool received_valid_message = false;
    for (ssize_t index = 0; index < size; ++index) {
      mavlink_message_t message{};
      if (mavlink_parse_char(MAVLINK_COMM_0, buffer[static_cast<size_t>(index)],
                             &message, &parser_status_)) {
        received_valid_message = true;
        handleMavlinkMessage(message);
      }
    }
    if (received_valid_message) {
      std::lock_guard<std::mutex> lock(peer_mutex_);
      peer_->value = source;
    }
  }
}

void MavlinkUdpGateway::handleMavlinkMessage(const mavlink_message_t &message) {
  noteNetworkActivity();
  if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    if (command.target_system != 0U &&
        command.target_system != config_.system_id)
      return;
    switch (command.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
      publishFlightCommand(command.param1 > 0.5F ? kFlightCommandArm
                                                 : kFlightCommandDisarm,
                           command.command, message.sysid, message.compid);
      break;
    case MAV_CMD_NAV_TAKEOFF:
      publishFlightCommand(kFlightCommandTakeoff, command.command,
                           message.sysid, message.compid);
      break;
    case MAV_CMD_NAV_LAND:
      publishFlightCommand(kFlightCommandLand, command.command, message.sysid,
                           message.compid);
      break;
    case MAV_CMD_DO_FLIGHTTERMINATION:
      publishFlightCommand(command.param1 > 0.5F ? kFlightCommandHalt
                                                 : kFlightCommandForceLand,
                           command.command, message.sysid, message.compid);
      break;
    case kMavCmdForceLand:
      publishFlightCommand(kFlightCommandForceLand, command.command,
                           message.sysid, message.compid);
      break;
    default:
      sendCommandAck(command.command, MAV_RESULT_UNSUPPORTED);
      break;
    }
  } else if (message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
    mavlink_set_position_target_local_ned_t target{};
    mavlink_msg_set_position_target_local_ned_decode(&message, &target);
    if (target.target_system != 0U && target.target_system != config_.system_id)
      return;

    PositionControlSetpoint setpoint{};
    setpoint.position[0] = target.y;
    setpoint.position[1] = target.x;
    setpoint.position[2] = -target.z;
    setpoint.velocity[0] = target.vy;
    setpoint.velocity[1] = target.vx;
    setpoint.velocity[2] = -target.vz;
    setpoint.acceleration[0] = target.afy;
    setpoint.acceleration[1] = target.afx;
    setpoint.acceleration[2] = -target.afz;
    setpoint.yaw = kPi * 0.5F - target.yaw;
    setpoint.yaw_rate = -target.yaw_rate;
    setpoint.active = 1U;
    setpoint.manual_control_allowed = 1U;

    Frame frame;
    setPayload(frame, MessageId::POSITION_CONTROL_SETPOINT, setpoint,
               ++frame_sequence_, nowMicros(), Source::MAVLINK,
               Reliability::BEST_EFFORT);
    dds_endpoint_.publish(frame);
  }
}

void MavlinkUdpGateway::publishFlightCommand(uint8_t command,
                                             uint32_t mavlink_command,
                                             uint8_t source_system,
                                             uint8_t source_component) {
  FlightCommand payload{};
  payload.command_id = mavlink_command;
  payload.issued_at_ms = static_cast<uint32_t>(nowMicros() / 1000U);
  payload.command = command;
  payload.source = static_cast<uint8_t>(CommandSource::MAVLINK);
  payload.target_system = source_system;
  payload.target_component = source_component;
  Frame frame;
  setPayload(frame, MessageId::FLIGHT_COMMAND, payload, ++frame_sequence_,
             nowMicros(), Source::MAVLINK, Reliability::RELIABLE);
  if (dds_endpoint_.publish(frame)) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      pending_mavlink_command_ = static_cast<uint16_t>(mavlink_command);
      pending_flight_command_ = command;
    }
    sendCommandAck(static_cast<uint16_t>(mavlink_command),
                   MAV_RESULT_IN_PROGRESS, 0U);
  } else
    sendCommandAck(static_cast<uint16_t>(mavlink_command),
                   MAV_RESULT_TEMPORARILY_REJECTED, 0U);
}

void MavlinkUdpGateway::noteNetworkActivity() {
  const uint64_t now_us = nowMicros();
  if (now_us - last_network_heartbeat_us_ < 250000U)
    return;
  last_network_heartbeat_us_ = now_us;
  Heartbeat heartbeat{};
  heartbeat.uptime_ms = static_cast<uint32_t>(now_us / 1000U);
  heartbeat.link_state = 3U;
  Frame frame;
  setPayload(frame, MessageId::NETWORK_HEARTBEAT, heartbeat, ++frame_sequence_,
             now_us, Source::MAVLINK, Reliability::BEST_EFFORT);
  (void)dds_endpoint_.publish(frame);
}

void MavlinkUdpGateway::sendCommandAck(uint16_t command, uint8_t result,
                                       uint8_t progress) {
  mavlink_message_t message{};
  mavlink_msg_command_ack_pack(config_.system_id, config_.component_id,
                               &message, command, result, progress, 0, 0U, 0U);
  sendMessage(message);
}

void MavlinkUdpGateway::sendMessage(mavlink_message_t &message) {
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
  const uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  std::lock_guard<std::mutex> peer_lock(peer_mutex_);
  if (socket_fd_ >= 0)
    sendto(socket_fd_, buffer.data(), size, 0,
           reinterpret_cast<const sockaddr *>(&peer_->value),
           sizeof(peer_->value));
}

uint64_t MavlinkUdpGateway::nowMicros() const {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

} // namespace aerial::vehicle
