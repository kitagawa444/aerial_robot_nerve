#include "rc/crsf_teleop.h"

namespace crsf
{

TeleopEvents TeleopInterpreter::update(
  const uint16_t* channels,
  std::size_t count,
  bool connected,
  uint32_t now_ms)
{
  TeleopEvents events;
  if (!connected || channels == nullptr)
  {
    reset();
    return events;
  }

  const bool arm_active = channel_high_(channels, count, channel_map_.arm);
  const bool takeoff_active =
    channel_high_(channels, count, channel_map_.takeoff_modifier) &&
    channel_high_(channels, count, channel_map_.takeoff_action);
  const bool land_active =
    channel_high_(channels, count, channel_map_.land_modifier) &&
    channel_high_(channels, count, channel_map_.land_action);
  const bool stop_active = channel_high_(channels, count, channel_map_.stop);

  if (!arm_active)
  {
    arm_release_seen_ = true;
  }
  if (!takeoff_active)
  {
    takeoff_release_seen_ = true;
  }
  if (!land_active)
  {
    land_release_seen_ = true;
  }
  if (!stop_active)
  {
    stop_release_seen_ = true;
    halt_sent_ = false;
  }

  events.arm = arm_release_seen_ && arm_active && !previous_arm_active_;
  events.takeoff =
    takeoff_release_seen_ && takeoff_active && !previous_takeoff_active_;
  events.land = land_release_seen_ && land_active && !previous_land_active_;

  const bool stop_rising =
    stop_release_seen_ && stop_active && !previous_stop_active_;
  if (stop_rising)
  {
    events.force_landing = true;
    stop_press_started_ms_ = now_ms;
  }
  if (stop_release_seen_ && stop_active && !halt_sent_ &&
      static_cast<uint32_t>(now_ms - stop_press_started_ms_) >=
        channel_map_.stop_to_halt_ms)
  {
    events.halt = true;
    halt_sent_ = true;
  }

  previous_arm_active_ = arm_active;
  previous_takeoff_active_ = takeoff_active;
  previous_land_active_ = land_active;
  previous_stop_active_ = stop_active;
  return events;
}

void TeleopInterpreter::reset()
{
  arm_release_seen_ = false;
  takeoff_release_seen_ = false;
  land_release_seen_ = false;
  stop_release_seen_ = false;
  previous_arm_active_ = false;
  previous_takeoff_active_ = false;
  previous_land_active_ = false;
  previous_stop_active_ = false;
  halt_sent_ = false;
  stop_press_started_ms_ = 0U;
}

bool TeleopInterpreter::channel_high_(
  const uint16_t* channels,
  std::size_t count,
  std::size_t index) const
{
  return index < count && channels[index] >= channel_map_.high_threshold;
}

}  // namespace crsf
