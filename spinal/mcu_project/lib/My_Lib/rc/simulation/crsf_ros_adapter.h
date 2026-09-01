#pragma once

#ifdef SIMULATION

#include "rc/crsf_input.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/u_int8.hpp>

class CrsfRosAdapterSim final
{
public:
  void init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, CrsfInput *input);
  void publish();

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  CrsfInput *input_{ nullptr };
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr teleop_command_pub_;
  uint32_t last_takeoff_sequence_{ 0U };
  uint32_t last_land_sequence_{ 0U };
  uint32_t last_command_sequence_{ 0U };

  void publishCommand_(crsf::TeleopCommand command);
};

#endif  // SIMULATION
