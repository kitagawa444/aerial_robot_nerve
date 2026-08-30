#pragma once

#include "rc/crsf_receiver.h"
#include "rc/crsf_teleop.h"
#include "ros_utils/ros_module_base.hpp"
#include "flight_control/flight_control.h"

#include <atomic>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>

class CrsfRosModule final : public RosModuleBase
{
public:
  CrsfRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(0)
        .max_publishers(4)
        .max_services(0)
        .max_timers(0))
  {}

  bool init_hw(
    UART_HandleTypeDef* huart,
    FlightControl* flight_control,
    osMutexId* control_mutex = nullptr);
  void create_entities(rcl_node_t& node) override;
  void update() override;
  void publish() override;

private:
  static constexpr uint32_t JOY_PUBLISH_INTERVAL_MS = 10U;
  static constexpr uint32_t STATUS_PUBLISH_INTERVAL_MS = 100U;
  static constexpr uint32_t ROS_COMMAND_MAX_AGE_MS = 100U;

  CrsfReceiver receiver_{};
  FlightControl* flight_control_{nullptr};
  osMutexId* control_mutex_{nullptr};

  rcl_publisher_t joy_pub_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t connected_pub_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t link_quality_pub_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t teleop_command_pub_ = rcl_get_zero_initialized_publisher();

  sensor_msgs__msg__Joy joy_msg_{};
  std_msgs__msg__Bool connected_msg_{};
  std_msgs__msg__UInt8 link_quality_msg_{};
  std_msgs__msg__UInt8 teleop_command_msg_{};
  float joy_axes_[crsf::CHANNEL_COUNT]{};
  crsf::TeleopInterpreter teleop_interpreter_{};
  std::atomic<uint32_t> pending_takeoff_ms_{0U};
  std::atomic<uint32_t> pending_land_ms_{0U};

  uint32_t last_published_rc_sequence_{0};
  uint32_t last_teleop_rc_sequence_{0};
  uint32_t last_joy_publish_ms_{0};
  uint32_t last_status_publish_ms_{0};
  bool last_published_connected_{false};
  bool status_published_{false};

  void apply_direct_command_(uint8_t command);
  void queue_ros_command_(std::atomic<uint32_t>& pending_stamp, uint32_t now_ms);
  void publish_pending_command_(
    std::atomic<uint32_t>& pending_stamp,
    crsf::TeleopCommand command,
    uint32_t now_ms);
  void lock_control_();
  void unlock_control_();
};
