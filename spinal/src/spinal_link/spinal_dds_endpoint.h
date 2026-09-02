#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "communication/spinal_link_protocol.h"
#include "communication/spinal_link_topics.generated.h"

namespace aerial::vehicle {

struct SpinalDdsStatistics {
  bool running{false};
  uint32_t matched_publications{0U};
  uint32_t matched_subscriptions{0U};
  uint64_t received_frames{0U};
  uint64_t transmitted_frames{0U};
  uint64_t receive_errors{0U};
  uint64_t transmit_errors{0U};
  uint64_t ack_received{0U};
  uint64_t ack_retries{0U};
  uint64_t ack_timeouts{0U};
  uint64_t duplicate_acks{0U};
  uint64_t rejected_acks{0U};
  uint32_t pending_requests{0U};
  uint32_t oldest_pending_age_ms{0U};
  uint32_t last_ack_rtt_ms{0U};
  uint16_t last_ack_message_id{0U};
  uint32_t last_ack_request_id{0U};
  uint64_t started_monotonic_ms{0U};
  uint64_t last_receive_monotonic_ms{0U};
  uint64_t last_transmit_monotonic_ms{0U};
  uint64_t last_ack_timeout_monotonic_ms{0U};
};

class SpinalDdsEndpoint {
public:
  using FrameCallback = std::function<void(const Frame &)>;
  enum class Role : uint8_t { HOST = 0, SPINAL_TEST = 1 };

  SpinalDdsEndpoint();
  ~SpinalDdsEndpoint();

  SpinalDdsEndpoint(const SpinalDdsEndpoint &) = delete;
  SpinalDdsEndpoint &operator=(const SpinalDdsEndpoint &) = delete;

  bool start(const std::string &participant_name, uint32_t domain_id = 0U,
             Role role = Role::HOST);
  void stop();
  bool publish(const Frame &frame);
  void setFrameCallback(FrameCallback callback);
  bool running() const;
  size_t pendingRequestCount() const;
  SpinalDdsStatistics statistics() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

} // namespace aerial::vehicle
