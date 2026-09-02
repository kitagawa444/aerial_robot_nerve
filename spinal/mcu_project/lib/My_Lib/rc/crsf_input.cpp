#include "rc/crsf_input.h"

#include "flight_control/flight_control.h"

void CrsfFlightControlSink::applyRcInput(float lateral, float forward, float vertical, float yaw, bool connected)
{
  if (flight_control_ == nullptr) return;

  PositionControlRcInput input;
  input.lateral = lateral;
  input.forward = forward;
  input.vertical = vertical;
  input.yaw = yaw;
  input.connected = connected;
  flight_control_->applyPositionControlRcInput(input);
}

bool CrsfFlightControlSink::requestFlightCommand(uint8_t command)
{
  return flight_control_ != nullptr && flight_control_->requestFlightCommand(command, FlightCommandSource::RC);
}

void CrsfInput::init(CrsfTransport *transport, CrsfControlSink *control_sink)
{
  transport_ = transport;
  control_sink_ = control_sink;
  parser_.reset();
  teleop_interpreter_.reset();
}

void CrsfInput::setEnabled(bool enabled)
{
  if (enabled_ == enabled) return;
  enabled_ = enabled;
  parser_.reset();
  teleop_interpreter_.reset();
  if (transport_ != nullptr) transport_->setEnabled(enabled);
  if (!enabled)
  {
    setConnected_(false);
    applyRcInput_(false);
  }
}

void CrsfInput::update(uint32_t now_ms)
{
  if (!enabled_ || transport_ == nullptr) return;

  transport_->update(now_ms);
  uint8_t buffer[READ_BUFFER_SIZE];
  std::size_t count = 0U;
  do
  {
    count = transport_->read(buffer, sizeof(buffer));
    for (std::size_t i = 0U; i < count; ++i)
    {
      const crsf::ParseResult result = parser_.process_byte(buffer[i]);
      if (result != crsf::ParseResult::None) handleParseResult_(result, now_ms);
    }
  } while (count == sizeof(buffer));

  if (snapshot_.connected && static_cast<uint32_t>(now_ms - snapshot_.last_rc_frame_ms) > crsf::SIGNAL_TIMEOUT_MS)
  {
    setConnected_(false);
    teleop_interpreter_.reset();
    applyRcInput_(false);
  }
}

bool CrsfInput::snapshot(Snapshot &output) const
{
  for (uint8_t attempt = 0U; attempt < 4U; ++attempt)
  {
    const uint32_t before = snapshot_version_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U) continue;
    output = snapshot_;
    const uint32_t after = snapshot_version_.load(std::memory_order_acquire);
    if (before == after) return true;
  }
  return false;
}

void CrsfInput::handleParseResult_(crsf::ParseResult result, uint32_t now_ms)
{
  snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_.valid_frame_count = parser_.valid_frame_count();
  snapshot_.crc_error_count = parser_.crc_error_count();

  if (result == crsf::ParseResult::RcChannels)
  {
    const crsf::RcChannels &channels = parser_.channels();
    for (std::size_t i = 0U; i < crsf::CHANNEL_COUNT; ++i)
    {
      snapshot_.raw[i] = channels.raw[i];
      snapshot_.normalized[i] = crsf::Parser::normalize_channel(channels.raw[i]);
    }
    snapshot_.last_rc_frame_ms = now_ms;
    snapshot_.connected = true;
    ++snapshot_.rc_frame_sequence;
  }
  else if (result == crsf::ParseResult::LinkStatistics)
  {
    snapshot_.link = parser_.link_statistics();
  }
  snapshot_version_.fetch_add(1U, std::memory_order_release);

  if (result == crsf::ParseResult::RcChannels) handleRcChannels_(now_ms);
}

void CrsfInput::handleRcChannels_(uint32_t now_ms)
{
  applyRcInput_(true);
  const crsf::TeleopEvents events = teleop_interpreter_.update(snapshot_.raw, crsf::CHANNEL_COUNT, true, now_ms);
  applyTeleopEvents_(events);
}

void CrsfInput::applyRcInput_(bool connected)
{
  if (control_sink_ == nullptr) return;
  control_sink_->applyRcInput(snapshot_.normalized[0], snapshot_.normalized[1], snapshot_.normalized[2],
                              snapshot_.normalized[3], connected);
}

void CrsfInput::applyTeleopEvents_(const crsf::TeleopEvents &events)
{
  if (control_sink_ == nullptr) return;
  if (events.arm) requestCommand_(FlightControlCommand::ARM_ON_CMD);
  if (events.takeoff)
  {
    requestCommand_(FlightControlCommand::TAKEOFF_CMD);
    snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
    ++snapshot_.takeoff_event_sequence;
    snapshot_version_.fetch_add(1U, std::memory_order_release);
  }
  if (events.land)
  {
    requestCommand_(FlightControlCommand::LAND_CMD);
    snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
    ++snapshot_.land_event_sequence;
    snapshot_version_.fetch_add(1U, std::memory_order_release);
  }
  if (events.force_landing) requestCommand_(FlightControlCommand::FORCE_LANDING_CMD);
  if (events.halt) requestCommand_(FlightControlCommand::HALT_CMD);
}

void CrsfInput::requestCommand_(uint8_t command)
{
  const bool accepted = control_sink_->requestFlightCommand(command);
  snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_.last_command = command;
  snapshot_.last_command_accepted = accepted;
  ++snapshot_.command_event_sequence;
  snapshot_version_.fetch_add(1U, std::memory_order_release);
}

void CrsfInput::setConnected_(bool connected)
{
  snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_.connected = connected;
  snapshot_version_.fetch_add(1U, std::memory_order_release);
}
