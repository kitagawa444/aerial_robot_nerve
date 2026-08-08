#include "rc/crsf_ros_module.h"

#include <rmw_microros/rmw_microros.h>

void CrsfRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();

  joy_msg_.axes.data = joy_axes_;
  joy_msg_.axes.size = crsf::CHANNEL_COUNT;
  joy_msg_.axes.capacity = crsf::CHANNEL_COUNT;
  joy_msg_.buttons.data = nullptr;
  joy_msg_.buttons.size = 0U;
  joy_msg_.buttons.capacity = 0U;

  (void)init_publisher_default(
    node,
    joy_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "rc/joy");

  (void)init_publisher_default(
    node,
    connected_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "rc/connected");

  (void)init_publisher_default(
    node,
    link_quality_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "rc/link_quality");

  last_joy_publish_ms_ = HAL_GetTick();
  last_status_publish_ms_ = last_joy_publish_ms_;
}

void CrsfRosModule::publish()
{
  if (ros_ready_ == nullptr || !ros_ready_->load(std::memory_order_acquire))
  {
    return;
  }

  CrsfReceiver::Snapshot snapshot{};
  if (!receiver_.snapshot(snapshot))
  {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  const bool connection_changed =
    !status_published_ || snapshot.connected != last_published_connected_;
  const bool status_due =
    connection_changed || (now_ms - last_status_publish_ms_) >= STATUS_PUBLISH_INTERVAL_MS;

  if (status_due)
  {
    connected_msg_.data = snapshot.connected;
    link_quality_msg_.data = snapshot.link.uplink_link_quality;
    (void)rcl_publish(&connected_pub_, &connected_msg_, nullptr);
    (void)rcl_publish(&link_quality_pub_, &link_quality_msg_, nullptr);
    last_published_connected_ = snapshot.connected;
    last_status_publish_ms_ = now_ms;
    status_published_ = true;
  }

  const bool new_rc_frame = snapshot.rc_frame_sequence != last_published_rc_sequence_;
  if (!snapshot.connected || !new_rc_frame ||
      (now_ms - last_joy_publish_ms_) < JOY_PUBLISH_INTERVAL_MS)
  {
    return;
  }

  for (std::size_t i = 0U; i < crsf::CHANNEL_COUNT; ++i)
  {
    joy_axes_[i] = snapshot.normalized[i];
  }

  const uint64_t epoch_ms = rmw_uros_epoch_millis();
  joy_msg_.header.stamp.sec = static_cast<int32_t>(epoch_ms / 1000ULL);
  joy_msg_.header.stamp.nanosec =
    static_cast<uint32_t>((epoch_ms % 1000ULL) * 1000000ULL);

  (void)rcl_publish(&joy_pub_, &joy_msg_, nullptr);
  last_published_rc_sequence_ = snapshot.rc_frame_sequence;
  last_joy_publish_ms_ = now_ms;
}
