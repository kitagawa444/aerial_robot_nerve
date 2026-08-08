#pragma once

#include "rc/crsf_receiver.h"
#include "ros_utils/ros_module_base.hpp"

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
        .max_publishers(3)
        .max_services(0)
        .max_timers(0))
  {}

  bool init_hw(UART_HandleTypeDef* huart) { return receiver_.init(huart); }
  void create_entities(rcl_node_t& node) override;
  void update() override { receiver_.update(); }
  void publish() override;

private:
  static constexpr uint32_t JOY_PUBLISH_INTERVAL_MS = 10U;
  static constexpr uint32_t STATUS_PUBLISH_INTERVAL_MS = 100U;

  CrsfReceiver receiver_{};

  rcl_publisher_t joy_pub_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t connected_pub_ = rcl_get_zero_initialized_publisher();
  rcl_publisher_t link_quality_pub_ = rcl_get_zero_initialized_publisher();

  sensor_msgs__msg__Joy joy_msg_{};
  std_msgs__msg__Bool connected_msg_{};
  std_msgs__msg__UInt8 link_quality_msg_{};
  float joy_axes_[crsf::CHANNEL_COUNT]{};

  uint32_t last_published_rc_sequence_{0};
  uint32_t last_joy_publish_ms_{0};
  uint32_t last_status_publish_ms_{0};
  bool last_published_connected_{false};
  bool status_published_{false};
};
