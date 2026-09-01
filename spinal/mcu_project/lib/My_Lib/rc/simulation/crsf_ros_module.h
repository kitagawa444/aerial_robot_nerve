#pragma once

#ifdef SIMULATION

#include "rc/crsf_protocol.h"
#include "rc/crsf_teleop.h"

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>

class FlightControl;

class CrsfRosModuleSim final
{
public:
  CrsfRosModuleSim() = default;
  ~CrsfRosModuleSim();

  void init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, const std::string &serial_port,
            uint32_t serial_baud, FlightControl *flight_control);
  void update();
  void setEnabled(bool enabled);

private:
  static constexpr uint32_t REOPEN_INTERVAL_MS = 1000U;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::string serial_port_;
  uint32_t serial_baud_{ crsf::DEFAULT_BAUD_RATE };
  int serial_fd_{ -1 };
  uint32_t last_open_attempt_ms_{ 0U };
  uint32_t last_rc_frame_ms_{ 0U };
  bool connected_{ false };
  bool enabled_{ true };
  FlightControl *flight_control_{ nullptr };

  crsf::Parser parser_{};
  crsf::TeleopInterpreter teleop_interpreter_{};

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr teleop_command_pub_;

  static uint32_t steady_time_ms_();
  bool open_serial_();
  void close_serial_();
  void handle_rc_frame_(uint32_t now_ms);
  void publish_events_(const crsf::TeleopEvents &events);
  void publish_command_(crsf::TeleopCommand command);
  void apply_direct_command_(uint8_t command);
  void apply_rc_input_(const crsf::RcChannels &channels, bool connected);
};

#endif  // SIMULATION
