#ifdef SIMULATION

#include "rc/simulation/crsf_posix_serial_transport.h"

#include <asm/termbits.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

CrsfPosixSerialTransport::~CrsfPosixSerialTransport() { closeSerial_(); }

void CrsfPosixSerialTransport::configure(const std::string &serial_port, uint32_t serial_baud)
{
  closeSerial_();
  serial_port_ = serial_port;
  serial_baud_ = serial_baud;
  last_open_attempt_ms_ = 0U;
}

void CrsfPosixSerialTransport::setEnabled(bool enabled)
{
  enabled_ = enabled;
  if (!enabled_) closeSerial_();
}

void CrsfPosixSerialTransport::update(uint32_t now_ms)
{
  if (!enabled_ || serial_port_.empty() || serial_fd_ >= 0) return;
  if (last_open_attempt_ms_ != 0U && static_cast<uint32_t>(now_ms - last_open_attempt_ms_) < REOPEN_INTERVAL_MS) return;
  last_open_attempt_ms_ = now_ms;
  (void)openSerial_();
}

std::size_t CrsfPosixSerialTransport::read(uint8_t *data, std::size_t capacity)
{
  if (serial_fd_ < 0 || data == nullptr || capacity == 0U) return 0U;
  const ssize_t count = ::read(serial_fd_, data, capacity);
  if (count > 0) return static_cast<std::size_t>(count);
  if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
  {
    last_error_ = errno;
    closeSerial_();
  }
  return 0U;
}

bool CrsfPosixSerialTransport::openSerial_()
{
  closeSerial_();
  serial_fd_ = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    last_error_ = errno;
    return false;
  }

  termios2 tty{};
  if (::ioctl(serial_fd_, TCGETS2, &tty) != 0)
  {
    last_error_ = errno;
    closeSerial_();
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
    last_error_ = errno;
    closeSerial_();
    return false;
  }

  last_error_ = 0;
  ++connection_sequence_;
  return true;
}

void CrsfPosixSerialTransport::closeSerial_()
{
  if (serial_fd_ >= 0)
  {
    (void)::close(serial_fd_);
    serial_fd_ = -1;
    ++connection_sequence_;
  }
}

#endif  // SIMULATION
