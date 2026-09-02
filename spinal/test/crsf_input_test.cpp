#include "rc/crsf_input.h"

#include "flight_control/flight_control_types.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
class FakeTransport final : public CrsfTransport
{
public:
  void setEnabled(bool enabled) override { enabled_ = enabled; }
  void update(uint32_t now_ms) override { (void)now_ms; }
  std::size_t read(uint8_t *data, std::size_t capacity) override
  {
    if (!enabled_) return 0U;
    std::size_t count = 0U;
    while (read_index_ < bytes_.size() && count < capacity) data[count++] = bytes_[read_index_++];
    return count;
  }

  void append(const std::vector<uint8_t> &bytes) { bytes_.insert(bytes_.end(), bytes.begin(), bytes.end()); }

private:
  std::vector<uint8_t> bytes_;
  std::size_t read_index_{ 0U };
  bool enabled_{ false };
};

class FakeControlSink final : public CrsfControlSink
{
public:
  void applyRcInput(float lateral, float forward, float vertical, float yaw, bool connected) override
  {
    axes = { lateral, forward, vertical, yaw };
    rc_connected = connected;
    ++rc_updates;
  }

  bool requestFlightCommand(uint8_t command) override
  {
    commands.push_back(command);
    return accept_commands;
  }

  std::array<float, 4U> axes{};
  std::vector<uint8_t> commands;
  uint32_t rc_updates{ 0U };
  bool rc_connected{ false };
  bool accept_commands{ true };
};

std::vector<uint8_t> makeRcFrame(const std::array<uint16_t, crsf::CHANNEL_COUNT> &channels)
{
  std::array<uint8_t, 22U> payload{};
  for (std::size_t channel = 0U; channel < channels.size(); ++channel)
  {
    const std::size_t bit_offset = channel * 11U;
    const std::size_t byte_offset = bit_offset / 8U;
    const uint8_t shift = static_cast<uint8_t>(bit_offset % 8U);
    const uint32_t packed = static_cast<uint32_t>(channels[channel]) << shift;
    payload[byte_offset] |= static_cast<uint8_t>(packed & 0xFFU);
    if (byte_offset + 1U < payload.size()) payload[byte_offset + 1U] |= static_cast<uint8_t>((packed >> 8U) & 0xFFU);
    if (byte_offset + 2U < payload.size()) payload[byte_offset + 2U] |= static_cast<uint8_t>((packed >> 16U) & 0xFFU);
  }

  std::vector<uint8_t> frame{ 0xC8U, 24U, crsf::FRAME_TYPE_RC_CHANNELS_PACKED };
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(crsf::Parser::crc8(&frame[2], 23U));
  return frame;
}

std::array<uint16_t, crsf::CHANNEL_COUNT> releasedChannels()
{
  std::array<uint16_t, crsf::CHANNEL_COUNT> channels{};
  channels.fill(992U);
  for (std::size_t i = 4U; i < channels.size(); ++i) channels[i] = 172U;
  return channels;
}
}  // namespace

TEST(CrsfInput, AppliesAxesWithoutRos)
{
  FakeTransport transport;
  FakeControlSink sink;
  CrsfInput input;
  input.init(&transport, &sink);
  input.setEnabled(true);

  auto channels = releasedChannels();
  channels[0] = 1812U;
  channels[1] = 172U;
  transport.append(makeRcFrame(channels));
  input.update(10U);

  EXPECT_TRUE(sink.rc_connected);
  EXPECT_EQ(sink.rc_updates, 1U);
  EXPECT_NEAR(sink.axes[0], 1.0F, 1.0e-6F);
  EXPECT_NEAR(sink.axes[1], -1.0F, 1.0e-6F);
}

TEST(CrsfInput, GeneratesOneShotSupervisorCommands)
{
  FakeTransport transport;
  FakeControlSink sink;
  CrsfInput input;
  input.init(&transport, &sink);
  input.setEnabled(true);

  auto channels = releasedChannels();
  transport.append(makeRcFrame(channels));
  input.update(10U);

  channels[4] = 1811U;
  transport.append(makeRcFrame(channels));
  input.update(20U);
  transport.append(makeRcFrame(channels));
  input.update(30U);

  ASSERT_EQ(sink.commands.size(), 1U);
  EXPECT_EQ(sink.commands.front(), FlightControlCommand::ARM_ON_CMD);
}

TEST(CrsfInput, DisconnectsAfterFrameTimeout)
{
  FakeTransport transport;
  FakeControlSink sink;
  CrsfInput input;
  input.init(&transport, &sink);
  input.setEnabled(true);

  transport.append(makeRcFrame(releasedChannels()));
  input.update(10U);
  ASSERT_TRUE(sink.rc_connected);

  input.update(10U + crsf::SIGNAL_TIMEOUT_MS + 1U);
  EXPECT_FALSE(sink.rc_connected);
  EXPECT_EQ(sink.rc_updates, 2U);
}

TEST(CrsfInput, DisablingTransportReleasesRcAuthority)
{
  FakeTransport transport;
  FakeControlSink sink;
  CrsfInput input;
  input.init(&transport, &sink);
  input.setEnabled(true);

  transport.append(makeRcFrame(releasedChannels()));
  input.update(10U);
  ASSERT_TRUE(sink.rc_connected);

  input.setEnabled(false);
  EXPECT_FALSE(sink.rc_connected);
  EXPECT_EQ(sink.rc_updates, 2U);
}

TEST(CrsfInput, ExposesRejectedCommandForOptionalDiagnostics)
{
  FakeTransport transport;
  FakeControlSink sink;
  sink.accept_commands = false;
  CrsfInput input;
  input.init(&transport, &sink);
  input.setEnabled(true);

  auto channels = releasedChannels();
  transport.append(makeRcFrame(channels));
  input.update(10U);
  channels[4] = 1811U;
  transport.append(makeRcFrame(channels));
  input.update(20U);

  CrsfInput::Snapshot snapshot{};
  ASSERT_TRUE(input.snapshot(snapshot));
  EXPECT_EQ(snapshot.command_event_sequence, 1U);
  EXPECT_EQ(snapshot.last_command, FlightControlCommand::ARM_ON_CMD);
  EXPECT_FALSE(snapshot.last_command_accepted);
}
