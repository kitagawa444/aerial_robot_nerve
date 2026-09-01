#include "rc/crsf_protocol.h"

namespace crsf
{

ParseResult Parser::process_byte(uint8_t byte)
{
  switch (state_)
  {
    case State::Address:
      // The destination address is not needed for RC decoding. CRC validation
      // on the body still protects against accepting a false frame boundary.
      (void)byte;
      state_ = State::Length;
      return ParseResult::None;

    case State::Length:
      if (byte < MIN_FRAME_LENGTH || byte > MAX_FRAME_LENGTH)
      {
        // Treat this byte as a possible next-frame address. Remaining in the
        // Length state makes the next byte the new candidate length.
        return ParseResult::None;
      }
      frame_length_ = byte;
      body_index_ = 0U;
      state_ = State::Body;
      return ParseResult::None;

    case State::Body:
      body_[body_index_++] = byte;
      if (body_index_ < frame_length_)
      {
        return ParseResult::None;
      }

      state_ = State::Address;
      return process_frame();
  }

  reset();
  return ParseResult::None;
}

void Parser::reset()
{
  state_ = State::Address;
  frame_length_ = 0U;
  body_index_ = 0U;
}

uint8_t Parser::crc8(const uint8_t* data, std::size_t size)
{
  uint8_t crc = 0U;
  while (size-- > 0U)
  {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x80U) != 0U
        ? static_cast<uint8_t>((crc << 1U) ^ 0xD5U)
        : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

float Parser::normalize_channel(uint16_t raw)
{
  // CRSF values 172..1811 correspond to the conventional 988..2012 us
  // control range, with 992 as the centre value.
  constexpr float centre = 992.0F;
  constexpr float half_range = 820.0F;
  float value = (static_cast<float>(raw) - centre) / half_range;
  if (value < -1.0F) value = -1.0F;
  if (value > 1.0F) value = 1.0F;
  return value;
}

ParseResult Parser::process_frame()
{
  const uint8_t expected_crc = body_[frame_length_ - 1U];
  const uint8_t actual_crc = crc8(body_, frame_length_ - 1U);
  if (actual_crc != expected_crc)
  {
    ++crc_error_count_;
    return ParseResult::CrcError;
  }

  ++valid_frame_count_;
  const uint8_t type = body_[0];
  const uint8_t payload_length = frame_length_ - 2U;

  if (type == FRAME_TYPE_RC_CHANNELS_PACKED &&
      payload_length == RC_PAYLOAD_LENGTH)
  {
    decode_channels(&body_[1]);
    return ParseResult::RcChannels;
  }

  if (type == FRAME_TYPE_LINK_STATISTICS &&
      payload_length >= LINK_STATISTICS_PAYLOAD_LENGTH)
  {
    decode_link_statistics(&body_[1]);
    return ParseResult::LinkStatistics;
  }

  return ParseResult::None;
}

void Parser::decode_channels(const uint8_t* payload)
{
  for (std::size_t channel = 0; channel < CHANNEL_COUNT; ++channel)
  {
    const std::size_t bit_offset = channel * 11U;
    const std::size_t byte_offset = bit_offset / 8U;
    const uint8_t shift = static_cast<uint8_t>(bit_offset % 8U);

    uint32_t packed = payload[byte_offset];
    if (byte_offset + 1U < RC_PAYLOAD_LENGTH)
    {
      packed |= static_cast<uint32_t>(payload[byte_offset + 1U]) << 8U;
    }
    if (byte_offset + 2U < RC_PAYLOAD_LENGTH)
    {
      packed |= static_cast<uint32_t>(payload[byte_offset + 2U]) << 16U;
    }
    channels_.raw[channel] = static_cast<uint16_t>((packed >> shift) & 0x07FFU);
  }
}

void Parser::decode_link_statistics(const uint8_t* payload)
{
  // CRSF carries RSSI as a positive magnitude. Expose conventional negative
  // dBm, selecting the stronger of the two receiver antennas.
  const uint8_t rssi_magnitude = payload[0] < payload[1] ? payload[0] : payload[1];
  link_statistics_.uplink_rssi_dbm = rssi_magnitude > 127U
    ? -127
    : static_cast<int8_t>(-static_cast<int16_t>(rssi_magnitude));
  link_statistics_.uplink_link_quality = payload[2];
  link_statistics_.uplink_snr_db = static_cast<int8_t>(payload[3]);
  link_statistics_.active_antenna = payload[4];
  link_statistics_.rf_mode = payload[5];
  link_statistics_.uplink_tx_power = payload[6];
  link_statistics_.downlink_rssi_dbm = payload[7] > 127U
    ? -127
    : static_cast<int8_t>(-static_cast<int16_t>(payload[7]));
  link_statistics_.downlink_link_quality = payload[8];
  link_statistics_.downlink_snr_db = static_cast<int8_t>(payload[9]);
}

}  // namespace crsf
