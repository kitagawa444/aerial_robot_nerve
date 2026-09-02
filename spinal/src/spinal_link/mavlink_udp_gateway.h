#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <common/mavlink.h>

#include "communication/spinal_link_protocol.h"
#include "spinal_dds_endpoint.h"

namespace aerial::vehicle {

class MavlinkUdpGateway {
public:
  struct Config {
    std::string bind_address{"0.0.0.0"};
    uint16_t bind_port{14550U};
    std::string remote_address{"255.255.255.255"};
    uint16_t remote_port{14550U};
    uint8_t system_id{1U};
    uint8_t component_id{MAV_COMP_ID_AUTOPILOT1};
  };

  explicit MavlinkUdpGateway(SpinalDdsEndpoint &dds_endpoint);
  ~MavlinkUdpGateway();

  bool start(const Config &config);
  void stop();
  void handleSpinalLinkFrame(const Frame &frame);
  bool running() const;

private:
  void receiveLoop();
  void handleMavlinkMessage(const mavlink_message_t &message);
  void sendMessage(mavlink_message_t &message);
  void sendCommandAck(uint16_t command, uint8_t result,
                      uint8_t progress = 100U);
  void noteNetworkActivity();
  void publishFlightCommand(uint8_t command, uint32_t mavlink_command,
                            uint8_t source_system, uint8_t source_component);
  uint64_t nowMicros() const;

  SpinalDdsEndpoint &dds_endpoint_;
  Config config_{};
  int socket_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  mutable std::mutex peer_mutex_;
  struct sockaddr_in_storage;
  std::unique_ptr<sockaddr_in_storage> peer_;
  uint32_t frame_sequence_{0U};
  std::mutex send_mutex_;
  mavlink_status_t parser_status_{};
  std::mutex command_mutex_;
  uint16_t pending_mavlink_command_{0U};
  uint8_t pending_flight_command_{0U};
  uint32_t last_command_status_sequence_{0U};
  uint64_t last_network_heartbeat_us_{0U};
};

} // namespace aerial::vehicle
