#pragma once

#include "rc/crsf_protocol.h"

#include <cstddef>
#include <cstdint>

namespace crsf
{

enum class TeleopCommand : uint8_t
{
  Arm = 1U,
  Takeoff = 2U,
  Land = 3U,
  ForceLanding = 4U,
  Halt = 5U,
};

struct TeleopEvents
{
  bool arm{false};
  bool takeoff{false};
  bool land{false};
  bool force_landing{false};
  bool halt{false};
};

struct TeleopChannelMap
{
  std::size_t arm{4U};
  std::size_t takeoff_modifier{5U};
  std::size_t takeoff_action{6U};
  std::size_t stop{7U};
  std::size_t land_modifier{8U};
  std::size_t land_action{9U};
  uint16_t high_threshold{1500U};
  uint32_t stop_to_halt_ms{1000U};
};

// Converts discrete CRSF channels into one-shot FC supervisor events.  An input
// must first be observed released after every link establishment.  This avoids
// arming because a button was held while the receiver or transmitter booted.
class TeleopInterpreter
{
public:
  explicit TeleopInterpreter(const TeleopChannelMap& channel_map = TeleopChannelMap())
  : channel_map_(channel_map)
  {}

  TeleopEvents update(
    const uint16_t* channels,
    std::size_t count,
    bool connected,
    uint32_t now_ms);
  void reset();

private:
  bool channel_high_(const uint16_t* channels, std::size_t count, std::size_t index) const;

  TeleopChannelMap channel_map_{};
  bool arm_release_seen_{false};
  bool takeoff_release_seen_{false};
  bool land_release_seen_{false};
  bool stop_release_seen_{false};
  bool previous_arm_active_{false};
  bool previous_takeoff_active_{false};
  bool previous_land_active_{false};
  bool previous_stop_active_{false};
  bool halt_sent_{false};
  uint32_t stop_press_started_ms_{0U};
};

}  // namespace crsf
