#pragma once

#ifdef SIMULATION

#include "rc/crsf_protocol.h"
#include "rc/crsf_transport.h"

#include <cstdint>
#include <string>

class CrsfPosixSerialTransport final : public CrsfTransport
{
public:
  CrsfPosixSerialTransport() = default;
  ~CrsfPosixSerialTransport() override;

  void configure(const std::string &serial_port, uint32_t serial_baud);
  void setEnabled(bool enabled) override;
  void update(uint32_t now_ms) override;
  std::size_t read(uint8_t *data, std::size_t capacity) override;

  bool isOpen() const { return serial_fd_ >= 0; }
  int lastError() const { return last_error_; }
  uint32_t connectionSequence() const { return connection_sequence_; }

private:
  static constexpr uint32_t REOPEN_INTERVAL_MS = 1000U;

  std::string serial_port_;
  uint32_t serial_baud_{ crsf::DEFAULT_BAUD_RATE };
  int serial_fd_{ -1 };
  int last_error_{ 0 };
  uint32_t last_open_attempt_ms_{ 0U };
  uint32_t connection_sequence_{ 0U };
  bool enabled_{ false };

  bool openSerial_();
  void closeSerial_();
};

#endif  // SIMULATION
