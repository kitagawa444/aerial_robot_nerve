#ifdef SIMULATION

#include "rc/simulation/crsf_ros_adapter.h"

void CrsfRosAdapterSim::init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, CrsfInput *input)
{
  node_ = node;
  input_ = input;
  if (node_)
    teleop_command_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("rc/teleop_command",
                                                                        rclcpp::QoS(10).reliable());
}

void CrsfRosAdapterSim::publish()
{
  if (input_ == nullptr || !teleop_command_pub_) return;
  CrsfInput::Snapshot snapshot{};
  if (!input_->snapshot(snapshot)) return;

  if (snapshot.takeoff_event_sequence != last_takeoff_sequence_)
  {
    publishCommand_(crsf::TeleopCommand::Takeoff);
    last_takeoff_sequence_ = snapshot.takeoff_event_sequence;
  }
  if (snapshot.land_event_sequence != last_land_sequence_)
  {
    publishCommand_(crsf::TeleopCommand::Land);
    last_land_sequence_ = snapshot.land_event_sequence;
  }
  if (snapshot.command_event_sequence != last_command_sequence_)
  {
    if (!snapshot.last_command_accepted)
      RCLCPP_WARN(node_->get_logger(), "Rejected direct CRSF flight command %u",
                  static_cast<unsigned int>(snapshot.last_command));
    last_command_sequence_ = snapshot.command_event_sequence;
  }
}

void CrsfRosAdapterSim::publishCommand_(crsf::TeleopCommand command)
{
  std_msgs::msg::UInt8 message;
  message.data = static_cast<uint8_t>(command);
  teleop_command_pub_->publish(message);
}

#endif  // SIMULATION
