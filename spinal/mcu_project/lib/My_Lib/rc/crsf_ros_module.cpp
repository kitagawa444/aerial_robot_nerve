#include "rc/crsf_ros_module.h"

#include <rmw_microros/rmw_microros.h>

bool CrsfRosModule::init_hw(UART_HandleTypeDef *huart, FlightControl *flight_control, osMutexId *control_mutex)
{
  flight_control_ = flight_control;
  control_mutex_ = control_mutex;
  return receiver_.init(huart);
}

void CrsfRosModule::update()
{
  receiver_.update();

  CrsfReceiver::Snapshot snapshot{};
  if (!receiver_.snapshot(snapshot)) return;

  const uint32_t now_ms = HAL_GetTick();
  if (snapshot.rc_frame_sequence != last_teleop_rc_sequence_)
  {
    apply_rc_input_(snapshot);
    const crsf::TeleopEvents events = teleop_interpreter_.update(snapshot.raw, crsf::CHANNEL_COUNT, snapshot.connected,
                                                                 now_ms);

    if (events.arm) apply_direct_command_(FlightControlCommand::ARM_ON_CMD);
    if (events.force_landing)
    {
      apply_direct_command_(FlightControlCommand::FORCE_LANDING_CMD);
    }
    if (events.halt) apply_direct_command_(FlightControlCommand::ARM_OFF_CMD);

    if (events.takeoff) queue_ros_command_(pending_takeoff_ms_, now_ms);
    if (events.land) queue_ros_command_(pending_land_ms_, now_ms);
    last_teleop_rc_sequence_ = snapshot.rc_frame_sequence;
  }
  else if (!snapshot.connected)
  {
    apply_rc_input_(snapshot);
    (void)teleop_interpreter_.update(nullptr, 0U, false, now_ms);
  }
}

void CrsfRosModule::create_entities(rcl_node_t &node)
{
  reserve_entities();

  joy_msg_.axes.data = joy_axes_;
  joy_msg_.axes.size = crsf::CHANNEL_COUNT;
  joy_msg_.axes.capacity = crsf::CHANNEL_COUNT;
  joy_msg_.buttons.data = nullptr;
  joy_msg_.buttons.size = 0U;
  joy_msg_.buttons.capacity = 0U;

  (void)init_publisher_default(node, joy_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy), "rc/joy");

  (void)init_publisher_default(node, connected_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rc/connected");

  (void)init_publisher_default(node, link_quality_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                               "rc/link_quality");

  (void)init_publisher_reliable(node, teleop_command_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                                "rc/teleop_command");

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
  const bool connection_changed = !status_published_ || snapshot.connected != last_published_connected_;
  const bool status_due = connection_changed || (now_ms - last_status_publish_ms_) >= STATUS_PUBLISH_INTERVAL_MS;

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
  publish_pending_command_(pending_takeoff_ms_, crsf::TeleopCommand::Takeoff, now_ms);
  publish_pending_command_(pending_land_ms_, crsf::TeleopCommand::Land, now_ms);

  if (!snapshot.connected || !new_rc_frame || (now_ms - last_joy_publish_ms_) < JOY_PUBLISH_INTERVAL_MS)
  {
    return;
  }

  for (std::size_t i = 0U; i < crsf::CHANNEL_COUNT; ++i)
  {
    joy_axes_[i] = snapshot.normalized[i];
  }

  const uint64_t epoch_ms = rmw_uros_epoch_millis();
  joy_msg_.header.stamp.sec = static_cast<int32_t>(epoch_ms / 1000ULL);
  joy_msg_.header.stamp.nanosec = static_cast<uint32_t>((epoch_ms % 1000ULL) * 1000000ULL);

  (void)rcl_publish(&joy_pub_, &joy_msg_, nullptr);
  last_published_rc_sequence_ = snapshot.rc_frame_sequence;
  last_joy_publish_ms_ = now_ms;
}

void CrsfRosModule::apply_direct_command_(uint8_t command)
{
  if (flight_control_ == nullptr) return;
  lock_control_();
  (void)flight_control_->applyFlightConfig(command);
  unlock_control_();
}

void CrsfRosModule::apply_rc_input_(const CrsfReceiver::Snapshot &snapshot)
{
  if (flight_control_ == nullptr) return;
  PositionControlRcInput input;
  input.lateral = snapshot.normalized[0];
  input.forward = snapshot.normalized[1];
  input.vertical = snapshot.normalized[2];
  input.yaw = snapshot.normalized[3];
  input.connected = snapshot.connected;
  lock_control_();
  flight_control_->applyPositionControlRcInput(input);
  unlock_control_();
}

void CrsfRosModule::queue_ros_command_(std::atomic<uint32_t> &pending_stamp, uint32_t now_ms)
{
  if (ros_ready_ == nullptr || !ros_ready_->load(std::memory_order_acquire)) return;
  pending_stamp.store(now_ms == 0U ? 1U : now_ms, std::memory_order_release);
}

void CrsfRosModule::publish_pending_command_(std::atomic<uint32_t> &pending_stamp, crsf::TeleopCommand command,
                                             uint32_t now_ms)
{
  const uint32_t stamp = pending_stamp.exchange(0U, std::memory_order_acq_rel);
  if (stamp == 0U || static_cast<uint32_t>(now_ms - stamp) > ROS_COMMAND_MAX_AGE_MS) return;

  teleop_command_msg_.data = static_cast<uint8_t>(command);
  (void)rcl_publish(&teleop_command_pub_, &teleop_command_msg_, nullptr);
}

void CrsfRosModule::lock_control_()
{
  if (control_mutex_ != nullptr && *control_mutex_ != nullptr)
  {
    osMutexWait(*control_mutex_, osWaitForever);
  }
}

void CrsfRosModule::unlock_control_()
{
  if (control_mutex_ != nullptr && *control_mutex_ != nullptr)
  {
    osMutexRelease(*control_mutex_);
  }
}
