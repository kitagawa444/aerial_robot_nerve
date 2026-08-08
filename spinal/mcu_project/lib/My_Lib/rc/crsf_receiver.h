#pragma once

#include <atomic>
#include <cstdint>

#include "config.h"
#include "rc/crsf_protocol.h"

class CrsfReceiver
{
public:
  static constexpr uint32_t DMA_BUFFER_SIZE = 256U;

  struct Snapshot
  {
    uint16_t raw[crsf::CHANNEL_COUNT]{};
    float normalized[crsf::CHANNEL_COUNT]{};
    crsf::LinkStatistics link{};
    bool connected{false};
    uint32_t rc_frame_sequence{0};
    uint32_t valid_frame_count{0};
    uint32_t crc_error_count{0};
    uint32_t last_rc_frame_ms{0};
  };

  bool init(UART_HandleTypeDef* huart);
  void update();
  bool snapshot(Snapshot& output) const;

private:
  int read_byte();
  void restart_dma();
  void update_snapshot(crsf::ParseResult result, uint32_t now_ms);
  void set_connected(bool connected);

  UART_HandleTypeDef* huart_{nullptr};
  uint32_t dma_read_index_{0};
  crsf::Parser parser_{};
  Snapshot snapshot_{};
  std::atomic<uint32_t> snapshot_version_{0};
};
