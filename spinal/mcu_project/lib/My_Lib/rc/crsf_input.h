#pragma once

#include "rc/crsf_protocol.h"
#include "rc/crsf_teleop.h"
#include "rc/crsf_transport.h"

#include <atomic>
#include <cstdint>

class FlightControl;

class CrsfControlSink
{
public:
  virtual ~CrsfControlSink() = default;

  virtual void applyRcInput(float lateral, float forward, float vertical, float yaw, bool connected) = 0;
  virtual bool requestFlightCommand(uint8_t command) = 0;
};

class CrsfFlightControlSink final : public CrsfControlSink
{
public:
  void init(FlightControl *flight_control) { flight_control_ = flight_control; }
  void applyRcInput(float lateral, float forward, float vertical, float yaw, bool connected) override;
  bool requestFlightCommand(uint8_t command) override;

private:
  FlightControl *flight_control_{ nullptr };
};

class CrsfInput
{
public:
  struct Snapshot
  {
    uint16_t raw[crsf::CHANNEL_COUNT]{};
    float normalized[crsf::CHANNEL_COUNT]{};
    crsf::LinkStatistics link{};
    bool connected{ false };
    uint32_t rc_frame_sequence{ 0U };
    uint32_t valid_frame_count{ 0U };
    uint32_t crc_error_count{ 0U };
    uint32_t last_rc_frame_ms{ 0U };
    uint32_t takeoff_event_sequence{ 0U };
    uint32_t land_event_sequence{ 0U };
    uint32_t command_event_sequence{ 0U };
    uint8_t last_command{ 0U };
    bool last_command_accepted{ false };
  };

  void init(CrsfTransport *transport, CrsfControlSink *control_sink);
  void setEnabled(bool enabled);
  void update(uint32_t now_ms);
  bool snapshot(Snapshot &output) const;

private:
  static constexpr std::size_t READ_BUFFER_SIZE = 256U;

  CrsfTransport *transport_{ nullptr };
  CrsfControlSink *control_sink_{ nullptr };
  crsf::Parser parser_{};
  crsf::TeleopInterpreter teleop_interpreter_{};
  Snapshot snapshot_{};
  std::atomic<uint32_t> snapshot_version_{ 0U };
  bool enabled_{ false };

  void handleParseResult_(crsf::ParseResult result, uint32_t now_ms);
  void handleRcChannels_(uint32_t now_ms);
  void applyRcInput_(bool connected);
  void applyTeleopEvents_(const crsf::TeleopEvents &events);
  void requestCommand_(uint8_t command);
  void setConnected_(bool connected);
};
