#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "spinal_dds_endpoint.h"

namespace aerial::vehicle {

enum class CommunicationLinkState : uint8_t {
  UNKNOWN = 0,
  DISCONNECTED = 1,
  CONNECTING = 2,
  CONNECTED = 3,
  STALE = 4,
  DISABLED = 5,
  ERROR = 6,
};

struct CommunicationLinkSnapshot {
  std::string name;
  CommunicationLinkState state{CommunicationLinkState::UNKNOWN};
  bool data_stale{true};
  bool interface_up{false};
  bool ip_assigned{false};
  bool transport_matched{false};
  uint32_t last_receive_age_ms{0U};
  float receive_rate_hz{0.0F};
  float transmit_rate_hz{0.0F};
  uint64_t received_count{0U};
  uint64_t transmitted_count{0U};
  uint64_t error_count{0U};
  int16_t rssi_dbm{0};
  uint8_t quality_percent{0U};
  std::string detail;
};

struct CommunicationAckSnapshot {
  CommunicationLinkState state{CommunicationLinkState::UNKNOWN};
  uint32_t pending_count{0U};
  uint32_t oldest_pending_age_ms{0U};
  uint64_t received_count{0U};
  uint64_t retry_count{0U};
  uint64_t timeout_count{0U};
  uint64_t duplicate_count{0U};
  uint64_t rejected_count{0U};
  uint32_t last_rtt_ms{0U};
  uint16_t last_message_id{0U};
  uint32_t last_request_id{0U};
  std::string detail;
};

struct CommunicationStatusSnapshot {
  CommunicationLinkSnapshot micro_usb;
  CommunicationLinkSnapshot ros;
  CommunicationLinkSnapshot wifi;
  CommunicationLinkSnapshot rc;
  CommunicationAckSnapshot ack;
};

class CommunicationMonitor {
public:
  explicit CommunicationMonitor(SpinalDdsEndpoint &endpoint);

  void setWifiInterface(const std::string &interface_name);
  void noteFlightStatus(uint8_t ros_state, uint8_t network_state,
                        uint8_t rc_state);
  void noteRcStatus(bool connected, uint8_t quality_percent, int8_t rssi_dbm);
  CommunicationStatusSnapshot sample();

private:
  static constexpr uint32_t kConnectedTimeoutMs = 1500U;
  static constexpr uint32_t kDisconnectedTimeoutMs = 5000U;

  static uint64_t monotonicMillis();
  static uint32_t ageMillis(uint64_t now_ms, uint64_t timestamp_ms);
  static CommunicationLinkState convertRemoteState(uint8_t state);
  static bool interfaceUp(const std::string &interface_name);
  static bool interfaceHasIp(const std::string &interface_name);

  CommunicationLinkSnapshot makeRemoteLink(const std::string &name,
                                           uint8_t remote_state,
                                           uint64_t report_time_ms,
                                           bool upstream_available,
                                           uint64_t now_ms) const;

  SpinalDdsEndpoint &endpoint_;
  std::mutex mutex_;
  std::string wifi_interface_;
  uint8_t ros_state_{0U};
  uint8_t network_state_{0U};
  uint8_t rc_state_{1U};
  uint64_t last_flight_status_ms_{0U};
  uint64_t last_rc_status_ms_{0U};
  uint8_t rc_quality_percent_{0U};
  int8_t rc_rssi_dbm_{0};
  SpinalDdsStatistics previous_statistics_{};
  uint64_t previous_sample_ms_{0U};
};

} // namespace aerial::vehicle
