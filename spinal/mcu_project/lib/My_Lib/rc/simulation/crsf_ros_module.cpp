#ifdef SIMULATION

#include "rc/simulation/crsf_ros_module.h"
#include "flight_control/flight_control.h"

#include <asm/termbits.h>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

CrsfRosModuleSim::~CrsfRosModuleSim() { close_serial_(); }

void CrsfRosModuleSim::init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                            const std::string &serial_port, uint32_t serial_baud, FlightControl *flight_control)
{
  node_ = node;
  serial_port_ = serial_port;
  serial_baud_ = serial_baud;
  flight_control_ = flight_control;

  if (!node_ || serial_port_.empty()) return;

  teleop_command_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("rc/teleop_command", rclcpp::QoS(10).reliable());

  last_open_attempt_ms_ = steady_time_ms_() - REOPEN_INTERVAL_MS;
  (void)open_serial_();
}

void CrsfRosModuleSim::update()
{
  if (!node_ || serial_port_.empty()) return;

  const uint32_t now_ms = steady_time_ms_();
  if (serial_fd_ < 0)
  {
    if (static_cast<uint32_t>(now_ms - last_open_attempt_ms_) >= REOPEN_INTERVAL_MS)
    {
      (void)open_serial_();
    }
    return;
  }

  uint8_t buffer[256];
  for (;;)
  {
    const ssize_t count = ::read(serial_fd_, buffer, sizeof(buffer));
    if (count > 0)
    {
      for (ssize_t i = 0; i < count; ++i)
      {
        if (parser_.process_byte(buffer[i]) == crsf::ParseResult::RcChannels)
        {
          handle_rc_frame_(now_ms);
        }
      }
      continue;
    }
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
      RCLCPP_ERROR(node_->get_logger(), "CRSF serial read failed on %s: errno=%d", serial_port_.c_str(), errno);
      close_serial_();
    }
    break;
  }

  if (connected_ && static_cast<uint32_t>(now_ms - last_rc_frame_ms_) > crsf::SIGNAL_TIMEOUT_MS)
  {
    connected_ = false;
    apply_rc_input_(parser_.channels(), false);
    (void)teleop_interpreter_.update(nullptr, 0U, false, now_ms);
  }
}

uint32_t CrsfRosModuleSim::steady_time_ms_()
{
  using namespace std::chrono;
  return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

bool CrsfRosModuleSim::open_serial_()
{
  last_open_attempt_ms_ = steady_time_ms_();
  close_serial_();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Cannot open CRSF serial port %s",
                         serial_port_.c_str());
    return false;
  }

  termios2 tty{};
  if (::ioctl(serial_fd_, TCGETS2, &tty) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "TCGETS2 failed for %s", serial_port_.c_str());
    close_serial_();
    return false;
  }

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cflag &= static_cast<unsigned int>(~(CBAUD | PARENB | CSTOPB | CSIZE | CRTSCTS));
  tty.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
  tty.c_ispeed = serial_baud_;
  tty.c_ospeed = serial_baud_;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::ioctl(serial_fd_, TCSETS2, &tty) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "TCSETS2 failed for %s", serial_port_.c_str());
    close_serial_();
    return false;
  }

  parser_.reset();
  teleop_interpreter_.reset();
  connected_ = false;
  RCLCPP_INFO(node_->get_logger(), "CRSF radio-in-the-loop input: %s at %u baud", serial_port_.c_str(), serial_baud_);
  return true;
}

void CrsfRosModuleSim::close_serial_()
{
  if (serial_fd_ >= 0)
  {
    (void)::close(serial_fd_);
    serial_fd_ = -1;
  }
  connected_ = false;
  teleop_interpreter_.reset();
  apply_rc_input_(parser_.channels(), false);
}

void CrsfRosModuleSim::handle_rc_frame_(uint32_t now_ms)
{
  connected_ = true;
  last_rc_frame_ms_ = now_ms;
  const crsf::RcChannels &channels = parser_.channels();
  apply_rc_input_(channels, true);
  publish_events_(teleop_interpreter_.update(channels.raw, crsf::CHANNEL_COUNT, true, now_ms));
}

void CrsfRosModuleSim::publish_events_(const crsf::TeleopEvents &events)
{
  if (events.arm) apply_direct_command_(FlightControlCommand::ARM_ON_CMD);
  if (events.takeoff)
  {
    apply_direct_command_(FlightControlCommand::TAKEOFF_CMD);
    publish_command_(crsf::TeleopCommand::Takeoff);
  }
  if (events.land)
  {
    apply_direct_command_(FlightControlCommand::LAND_CMD);
    publish_command_(crsf::TeleopCommand::Land);
  }
  if (events.force_landing)
  {
    apply_direct_command_(FlightControlCommand::FORCE_LANDING_CMD);
  }
  if (events.halt) apply_direct_command_(FlightControlCommand::HALT_CMD);
}

void CrsfRosModuleSim::publish_command_(crsf::TeleopCommand command)
{
  if (!teleop_command_pub_) return;
  std_msgs::msg::UInt8 message;
  message.data = static_cast<uint8_t>(command);
  teleop_command_pub_->publish(message);
}

void CrsfRosModuleSim::apply_direct_command_(uint8_t command)
{
  if (flight_control_ == nullptr) return;
  if (!flight_control_->requestFlightCommand(command, FlightCommandSource::RC))
  {
    RCLCPP_WARN(node_->get_logger(), "Rejected direct CRSF flight command %u", static_cast<unsigned int>(command));
  }
}

void CrsfRosModuleSim::apply_rc_input_(const crsf::RcChannels &channels, bool connected)
{
  if (flight_control_ == nullptr) return;
  PositionControlRcInput input;
  input.lateral = crsf::Parser::normalize_channel(channels.raw[0]);
  input.forward = crsf::Parser::normalize_channel(channels.raw[1]);
  input.vertical = crsf::Parser::normalize_channel(channels.raw[2]);
  input.yaw = crsf::Parser::normalize_channel(channels.raw[3]);
  input.connected = connected;
  flight_control_->applyPositionControlRcInput(input);
}

#endif  // SIMULATION
