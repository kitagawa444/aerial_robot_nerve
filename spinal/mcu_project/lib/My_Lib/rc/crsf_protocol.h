#pragma once

#include <cstddef>
#include <cstdint>

namespace crsf
{

constexpr std::size_t CHANNEL_COUNT = 16;
constexpr uint32_t DEFAULT_BAUD_RATE = 420000U;
constexpr uint32_t SIGNAL_TIMEOUT_MS = 100U;

constexpr uint8_t FRAME_TYPE_LINK_STATISTICS = 0x14U;
constexpr uint8_t FRAME_TYPE_RC_CHANNELS_PACKED = 0x16U;

struct RcChannels
{
  uint16_t raw[CHANNEL_COUNT]{};
};

struct LinkStatistics
{
  int8_t uplink_rssi_dbm{0};
  uint8_t uplink_link_quality{0};
  int8_t uplink_snr_db{0};
  uint8_t active_antenna{0};
  uint8_t rf_mode{0};
  uint8_t uplink_tx_power{0};
  int8_t downlink_rssi_dbm{0};
  uint8_t downlink_link_quality{0};
  int8_t downlink_snr_db{0};
};

enum class ParseResult : uint8_t
{
  None,
  RcChannels,
  LinkStatistics,
  CrcError,
};

class Parser
{
public:
  ParseResult process_byte(uint8_t byte);
  void reset();

  const RcChannels& channels() const { return channels_; }
  const LinkStatistics& link_statistics() const { return link_statistics_; }

  uint32_t valid_frame_count() const { return valid_frame_count_; }
  uint32_t crc_error_count() const { return crc_error_count_; }

  static uint8_t crc8(const uint8_t* data, std::size_t size);
  static float normalize_channel(uint16_t raw);

private:
  static constexpr uint8_t MIN_FRAME_LENGTH = 2U;
  static constexpr uint8_t MAX_FRAME_LENGTH = 62U;
  static constexpr uint8_t RC_PAYLOAD_LENGTH = 22U;
  static constexpr uint8_t LINK_STATISTICS_PAYLOAD_LENGTH = 10U;

  enum class State : uint8_t
  {
    Address,
    Length,
    Body,
  };

  ParseResult process_frame();
  void decode_channels(const uint8_t* payload);
  void decode_link_statistics(const uint8_t* payload);

  State state_{State::Address};
  uint8_t frame_length_{0};
  uint8_t body_index_{0};
  uint8_t body_[MAX_FRAME_LENGTH]{};

  RcChannels channels_{};
  LinkStatistics link_statistics_{};
  uint32_t valid_frame_count_{0};
  uint32_t crc_error_count_{0};
};

}  // namespace crsf
