#include "rc/crsf_protocol.h"

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

namespace
{

using Frame = std::array<uint8_t, 26>;

Frame make_rc_frame(const std::array<uint16_t, crsf::CHANNEL_COUNT>& channels)
{
  Frame frame{};
  frame[0] = 0xC8U;
  frame[1] = 24U;
  frame[2] = crsf::FRAME_TYPE_RC_CHANNELS_PACKED;

  for (std::size_t channel = 0; channel < channels.size(); ++channel)
  {
    const std::size_t bit_offset = channel * 11U;
    const std::size_t byte_offset = 3U + bit_offset / 8U;
    const uint8_t shift = static_cast<uint8_t>(bit_offset % 8U);
    const uint32_t value = static_cast<uint32_t>(channels[channel] & 0x07FFU) << shift;
    frame[byte_offset] |= static_cast<uint8_t>(value & 0xFFU);
    if (byte_offset + 1U < 25U)
    {
      frame[byte_offset + 1U] |= static_cast<uint8_t>((value >> 8U) & 0xFFU);
    }
    if (byte_offset + 2U < 25U)
    {
      frame[byte_offset + 2U] |= static_cast<uint8_t>((value >> 16U) & 0xFFU);
    }
  }

  frame[25] = crsf::Parser::crc8(&frame[2], 23U);
  return frame;
}

template<std::size_t Size>
crsf::ParseResult feed(crsf::Parser& parser, const std::array<uint8_t, Size>& frame)
{
  crsf::ParseResult result = crsf::ParseResult::None;
  for (uint8_t byte : frame)
  {
    const crsf::ParseResult current = parser.process_byte(byte);
    if (current != crsf::ParseResult::None)
    {
      result = current;
    }
  }
  return result;
}

TEST(CrsfProtocol, DecodesPackedChannels)
{
  const std::array<uint16_t, crsf::CHANNEL_COUNT> expected{
    172U, 992U, 1811U, 500U, 1200U, 42U, 2047U, 0U,
    100U, 200U, 300U, 400U, 600U, 800U, 1000U, 1600U};
  const Frame frame = make_rc_frame(expected);

  crsf::Parser parser;
  ASSERT_EQ(feed(parser, frame), crsf::ParseResult::RcChannels);
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(parser.channels().raw[i], expected[i]) << "channel " << i;
  }
  EXPECT_EQ(parser.valid_frame_count(), 1U);
  EXPECT_EQ(parser.crc_error_count(), 0U);
}

TEST(CrsfProtocol, RejectsBadCrcAndRecoversOnNextFrame)
{
  const std::array<uint16_t, crsf::CHANNEL_COUNT> channels{};
  Frame bad_frame = make_rc_frame(channels);
  bad_frame[25] ^= 0x01U;
  const Frame good_frame = make_rc_frame(channels);

  crsf::Parser parser;
  EXPECT_EQ(feed(parser, bad_frame), crsf::ParseResult::CrcError);
  EXPECT_EQ(feed(parser, good_frame), crsf::ParseResult::RcChannels);
  EXPECT_EQ(parser.valid_frame_count(), 1U);
  EXPECT_EQ(parser.crc_error_count(), 1U);
}

TEST(CrsfProtocol, NormalizesConventionalControlRange)
{
  EXPECT_FLOAT_EQ(crsf::Parser::normalize_channel(172U), -1.0F);
  EXPECT_FLOAT_EQ(crsf::Parser::normalize_channel(992U), 0.0F);
  EXPECT_FLOAT_EQ(crsf::Parser::normalize_channel(1812U), 1.0F);
  EXPECT_FLOAT_EQ(crsf::Parser::normalize_channel(0U), -1.0F);
  EXPECT_FLOAT_EQ(crsf::Parser::normalize_channel(2047U), 1.0F);
}

TEST(CrsfProtocol, DecodesLinkStatistics)
{
  std::array<uint8_t, 14> frame{
    0xC8U, 12U, crsf::FRAME_TYPE_LINK_STATISTICS,
    70U, 80U, 95U, static_cast<uint8_t>(-5), 1U,
    3U, 4U, 90U, 88U, 7U, 0U};
  frame.back() = crsf::Parser::crc8(&frame[2], 11U);

  crsf::Parser parser;
  ASSERT_EQ(feed(parser, frame), crsf::ParseResult::LinkStatistics);
  const crsf::LinkStatistics& link = parser.link_statistics();
  EXPECT_EQ(link.uplink_rssi_dbm, -70);
  EXPECT_EQ(link.uplink_link_quality, 95U);
  EXPECT_EQ(link.uplink_snr_db, -5);
  EXPECT_EQ(link.active_antenna, 1U);
  EXPECT_EQ(link.rf_mode, 3U);
  EXPECT_EQ(link.uplink_tx_power, 4U);
  EXPECT_EQ(link.downlink_rssi_dbm, -90);
  EXPECT_EQ(link.downlink_link_quality, 88U);
  EXPECT_EQ(link.downlink_snr_db, 7);
}

}  // namespace
