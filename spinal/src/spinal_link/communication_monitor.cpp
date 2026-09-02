#include "communication_monitor.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <ifaddrs.h>
#include <limits>
#include <net/if.h>
#include <sys/socket.h>

namespace aerial::vehicle {

CommunicationMonitor::CommunicationMonitor(SpinalDdsEndpoint &endpoint)
    : endpoint_(endpoint) {}

void CommunicationMonitor::setWifiInterface(const std::string &interface_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  wifi_interface_ = interface_name;
}

void CommunicationMonitor::noteFlightStatus(uint8_t ros_state,
                                            uint8_t network_state,
                                            uint8_t rc_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  ros_state_ = ros_state;
  network_state_ = network_state;
  rc_state_ = rc_state;
  last_flight_status_ms_ = monotonicMillis();
}

void CommunicationMonitor::noteRcStatus(bool connected, uint8_t quality_percent,
                                        int8_t rssi_dbm) {
  std::lock_guard<std::mutex> lock(mutex_);
  rc_state_ = connected ? 3U : 1U;
  rc_quality_percent_ = quality_percent;
  rc_rssi_dbm_ = rssi_dbm;
  last_rc_status_ms_ = monotonicMillis();
}

uint64_t CommunicationMonitor::monotonicMillis() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

uint32_t CommunicationMonitor::ageMillis(uint64_t now_ms,
                                         uint64_t timestamp_ms) {
  if (timestamp_ms == 0U || timestamp_ms > now_ms)
    return std::numeric_limits<uint32_t>::max();
  return static_cast<uint32_t>(std::min<uint64_t>(
      now_ms - timestamp_ms, std::numeric_limits<uint32_t>::max()));
}

CommunicationLinkState CommunicationMonitor::convertRemoteState(uint8_t state) {
  return state <= static_cast<uint8_t>(CommunicationLinkState::DISABLED)
             ? static_cast<CommunicationLinkState>(state)
             : CommunicationLinkState::ERROR;
}

bool CommunicationMonitor::interfaceUp(const std::string &interface_name) {
  if (interface_name.empty())
    return false;
  std::ifstream state("/sys/class/net/" + interface_name + "/operstate");
  std::string value;
  state >> value;
  return value == "up" || value == "unknown";
}

bool CommunicationMonitor::interfaceHasIp(const std::string &interface_name) {
  if (interface_name.empty())
    return false;
  ifaddrs *addresses = nullptr;
  if (getifaddrs(&addresses) != 0)
    return false;
  bool found = false;
  for (ifaddrs *address = addresses; address != nullptr;
       address = address->ifa_next) {
    if (address->ifa_addr != nullptr && interface_name == address->ifa_name &&
        address->ifa_addr->sa_family == AF_INET &&
        (address->ifa_flags & IFF_UP) != 0U) {
      found = true;
      break;
    }
  }
  freeifaddrs(addresses);
  return found;
}

CommunicationLinkSnapshot CommunicationMonitor::makeRemoteLink(
    const std::string &name, uint8_t remote_state, uint64_t report_time_ms,
    bool upstream_available, uint64_t now_ms) const {
  CommunicationLinkSnapshot result;
  result.name = name;
  result.last_receive_age_ms = ageMillis(now_ms, report_time_ms);
  result.data_stale =
      !upstream_available || result.last_receive_age_ms > kConnectedTimeoutMs;
  if (!upstream_available ||
      result.last_receive_age_ms > kDisconnectedTimeoutMs) {
    result.state = CommunicationLinkState::UNKNOWN;
    result.detail = "Spinal status unavailable";
  } else {
    result.state = convertRemoteState(remote_state);
    result.detail =
        result.data_stale ? "Spinal status is stale" : "Reported by Spinal";
  }
  return result;
}

CommunicationStatusSnapshot CommunicationMonitor::sample() {
  const uint64_t now_ms = monotonicMillis();
  const SpinalDdsStatistics statistics = endpoint_.statistics();
  CommunicationStatusSnapshot result;

  result.micro_usb.name = "Micro-USB / XRCE-DDS";
  result.micro_usb.transport_matched = statistics.matched_publications > 0U &&
                                       statistics.matched_subscriptions > 0U;
  result.micro_usb.interface_up = statistics.running;
  result.micro_usb.ip_assigned = false;
  result.micro_usb.received_count = statistics.received_frames;
  result.micro_usb.transmitted_count = statistics.transmitted_frames;
  result.micro_usb.error_count =
      statistics.receive_errors + statistics.transmit_errors;
  result.micro_usb.last_receive_age_ms =
      ageMillis(now_ms, statistics.last_receive_monotonic_ms);
  result.micro_usb.data_stale =
      result.micro_usb.last_receive_age_ms > kConnectedTimeoutMs;
  if (!statistics.running) {
    result.micro_usb.state = CommunicationLinkState::DISABLED;
    result.micro_usb.detail = "DDS endpoint stopped";
  } else if (!result.micro_usb.transport_matched) {
    const uint32_t start_age =
        ageMillis(now_ms, statistics.started_monotonic_ms);
    result.micro_usb.state = start_age <= kDisconnectedTimeoutMs
                                 ? CommunicationLinkState::CONNECTING
                                 : CommunicationLinkState::DISCONNECTED;
    result.micro_usb.detail = "Waiting for XRCE-DDS entities";
  } else if (result.micro_usb.last_receive_age_ms <= kConnectedTimeoutMs) {
    result.micro_usb.state = CommunicationLinkState::CONNECTED;
    result.micro_usb.detail = "DDS matched; Spinal traffic active";
  } else if (result.micro_usb.last_receive_age_ms <= kDisconnectedTimeoutMs) {
    result.micro_usb.state = CommunicationLinkState::STALE;
    result.micro_usb.detail = "Spinal traffic is stale";
  } else {
    result.micro_usb.state = CommunicationLinkState::DISCONNECTED;
    result.micro_usb.detail = "Spinal traffic timed out";
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const float elapsed_seconds =
        previous_sample_ms_ == 0U || now_ms <= previous_sample_ms_
            ? 0.0F
            : static_cast<float>(now_ms - previous_sample_ms_) / 1000.0F;
    if (elapsed_seconds > 0.0F) {
      const uint64_t received_delta =
          statistics.received_frames >= previous_statistics_.received_frames
              ? statistics.received_frames -
                    previous_statistics_.received_frames
              : statistics.received_frames;
      const uint64_t transmitted_delta =
          statistics.transmitted_frames >=
                  previous_statistics_.transmitted_frames
              ? statistics.transmitted_frames -
                    previous_statistics_.transmitted_frames
              : statistics.transmitted_frames;
      result.micro_usb.receive_rate_hz =
          static_cast<float>(received_delta) / elapsed_seconds;
      result.micro_usb.transmit_rate_hz =
          static_cast<float>(transmitted_delta) / elapsed_seconds;
    }
    previous_statistics_ = statistics;
    previous_sample_ms_ = now_ms;

    const bool upstream_available =
        result.micro_usb.state == CommunicationLinkState::CONNECTED ||
        result.micro_usb.state == CommunicationLinkState::STALE;
    result.ros =
        makeRemoteLink("ROS 2 bridge", ros_state_, last_flight_status_ms_,
                       upstream_available, now_ms);
    result.wifi =
        makeRemoteLink("Wi-Fi / MAVLink GCS", network_state_,
                       last_flight_status_ms_, upstream_available, now_ms);
    result.wifi.interface_up = interfaceUp(wifi_interface_);
    result.wifi.ip_assigned = interfaceHasIp(wifi_interface_);
    if (!wifi_interface_.empty()) {
      if (!result.wifi.interface_up) {
        result.wifi.state = CommunicationLinkState::DISCONNECTED;
        result.wifi.detail = wifi_interface_ + " is down";
      } else if (!result.wifi.ip_assigned) {
        result.wifi.state = CommunicationLinkState::CONNECTING;
        result.wifi.detail = wifi_interface_ + " has no IPv4 address";
      } else {
        result.wifi.detail += "; " + wifi_interface_ + " is up";
      }
    }

    result.rc = makeRemoteLink("ELRS / CRSF", rc_state_, last_flight_status_ms_,
                               upstream_available, now_ms);
    result.rc.last_receive_age_ms = ageMillis(now_ms, last_rc_status_ms_);
    result.rc.quality_percent = rc_quality_percent_;
    result.rc.rssi_dbm = rc_rssi_dbm_;
    if (last_rc_status_ms_ != 0U && upstream_available)
      result.rc.detail = "Reported by Spinal CRSF receiver";
  }

  result.ack.pending_count = statistics.pending_requests;
  result.ack.oldest_pending_age_ms = statistics.oldest_pending_age_ms;
  result.ack.received_count = statistics.ack_received;
  result.ack.retry_count = statistics.ack_retries;
  result.ack.timeout_count = statistics.ack_timeouts;
  result.ack.duplicate_count = statistics.duplicate_acks;
  result.ack.rejected_count = statistics.rejected_acks;
  result.ack.last_rtt_ms = statistics.last_ack_rtt_ms;
  result.ack.last_message_id = statistics.last_ack_message_id;
  result.ack.last_request_id = statistics.last_ack_request_id;
  const uint32_t timeout_age =
      ageMillis(now_ms, statistics.last_ack_timeout_monotonic_ms);
  if (!result.micro_usb.transport_matched) {
    result.ack.state = CommunicationLinkState::UNKNOWN;
    result.ack.detail = "Spinal Link is not matched";
  } else if (timeout_age <= kDisconnectedTimeoutMs) {
    result.ack.state = CommunicationLinkState::ERROR;
    result.ack.detail = "Recent request timed out";
  } else if (statistics.pending_requests > 0U &&
             statistics.oldest_pending_age_ms > 500U) {
    result.ack.state = CommunicationLinkState::STALE;
    result.ack.detail = "Waiting for ACK";
  } else {
    result.ack.state = CommunicationLinkState::CONNECTED;
    result.ack.detail = statistics.pending_requests == 0U
                            ? "No pending requests"
                            : "Request in progress";
  }
  return result;
}

} // namespace aerial::vehicle
