#pragma once

#include <cstddef>

#include "communication/spinal_link_protocol.h"

namespace aerial {
namespace vehicle {

template <size_t Capacity> class RequestDeduplicator {
public:
  bool seen(const Frame &frame) const {
    for (const auto &request : requests_) {
      if (request.valid && request.request_id == frame.header.request_id &&
          request.message_id == frame.header.message_id &&
          request.source == frame.header.source)
        return true;
    }
    return false;
  }

  void remember(const Frame &frame) {
    Entry &request = requests_[next_];
    request.request_id = frame.header.request_id;
    request.message_id = frame.header.message_id;
    request.source = frame.header.source;
    request.valid = true;
    next_ = (next_ + 1U) % Capacity;
  }

private:
  static_assert(Capacity > 0U, "Deduplication cache must not be empty");
  struct Entry {
    uint32_t request_id{0U};
    uint16_t message_id{0U};
    uint8_t source{0U};
    bool valid{false};
  };
  Entry requests_[Capacity]{};
  size_t next_{0U};
};

} // namespace vehicle
} // namespace aerial
