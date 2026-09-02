#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "mavlink_udp_gateway.h"
#include "spinal_dds_endpoint.h"

namespace {
std::atomic<bool> running{true};

void stopHandler(int) { running.store(false); }

uint32_t environmentUnsigned(const char *name, uint32_t fallback) {
  const char *value = std::getenv(name);
  if (value == nullptr)
    return fallback;
  try {
    return static_cast<uint32_t>(std::stoul(value));
  } catch (...) {
    return fallback;
  }
}

std::string environmentString(const char *name, const std::string &fallback) {
  const char *value = std::getenv(name);
  return value == nullptr ? fallback : value;
}
} // namespace

int main() {
  std::signal(SIGINT, stopHandler);
  std::signal(SIGTERM, stopHandler);

  aerial::vehicle::SpinalDdsEndpoint endpoint;
  const uint32_t domain_id =
      environmentUnsigned("SPINAL_LINK_DDS_DOMAIN_ID", 0U);
  if (!endpoint.start("gcs_gateway", domain_id)) {
    std::cerr << "Failed to start the vehicle DDS endpoint" << std::endl;
    return 1;
  }

  aerial::vehicle::MavlinkUdpGateway gateway(endpoint);
  aerial::vehicle::MavlinkUdpGateway::Config config;
  config.bind_address = environmentString("MAVLINK_BIND_ADDRESS", "0.0.0.0");
  config.bind_port =
      static_cast<uint16_t>(environmentUnsigned("MAVLINK_BIND_PORT", 14550U));
  config.remote_address =
      environmentString("MAVLINK_REMOTE_ADDRESS", "255.255.255.255");
  config.remote_port =
      static_cast<uint16_t>(environmentUnsigned("MAVLINK_REMOTE_PORT", 14550U));
  config.system_id =
      static_cast<uint8_t>(environmentUnsigned("MAVLINK_SYSTEM_ID", 1U));
  config.component_id = static_cast<uint8_t>(
      environmentUnsigned("MAVLINK_COMPONENT_ID", MAV_COMP_ID_AUTOPILOT1));
  if (!gateway.start(config)) {
    std::cerr << "Failed to open MAVLink UDP " << config.bind_address << ':'
              << config.bind_port << std::endl;
    return 2;
  }

  endpoint.setFrameCallback([&gateway](const aerial::vehicle::Frame &frame) {
    gateway.handleSpinalLinkFrame(frame);
  });
  std::cout << "GCS Gateway: Spinal Link DDS domain " << domain_id
            << ", MAVLink UDP "
            << config.bind_address << ':' << config.bind_port << " -> "
            << config.remote_address << ':' << config.remote_port << std::endl;

  while (running.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gateway.stop();
  endpoint.stop();
  return 0;
}
