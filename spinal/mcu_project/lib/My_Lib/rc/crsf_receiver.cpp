#include "rc/crsf_receiver.h"

#include <cstring>

namespace
{
#ifdef STM32H7
uint8_t crsf_dma_buffer[CrsfReceiver::DMA_BUFFER_SIZE]
  __attribute__((section(".CrsfRxBufferSection")));
#else
uint8_t crsf_dma_buffer[CrsfReceiver::DMA_BUFFER_SIZE];
#endif
}

bool CrsfReceiver::init(UART_HandleTypeDef* huart)
{
  if (huart == nullptr || huart->hdmarx == nullptr)
  {
    return false;
  }

  huart_ = huart;
  dma_read_index_ = 0U;
  parser_.reset();
  std::memset(crsf_dma_buffer, 0, sizeof(crsf_dma_buffer));

  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  return HAL_UART_Receive_DMA(
    huart_, crsf_dma_buffer, static_cast<uint16_t>(DMA_BUFFER_SIZE)) == HAL_OK;
}

void CrsfReceiver::update()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr)
  {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  int byte = -1;
  while ((byte = read_byte()) >= 0)
  {
    const crsf::ParseResult result = parser_.process_byte(static_cast<uint8_t>(byte));
    if (result != crsf::ParseResult::None)
    {
      update_snapshot(result, now_ms);
    }
  }

  if (snapshot_.connected &&
      (now_ms - snapshot_.last_rc_frame_ms) > crsf::SIGNAL_TIMEOUT_MS)
  {
    set_connected(false);
  }
}

bool CrsfReceiver::snapshot(Snapshot& output) const
{
  // A small seqlock keeps the core task (producer) independent from the ROS
  // task (consumer), without taking the ROS mutex in the UART path.
  for (uint8_t attempt = 0U; attempt < 4U; ++attempt)
  {
    const uint32_t before = snapshot_version_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U)
    {
      continue;
    }
    output = snapshot_;
    const uint32_t after = snapshot_version_.load(std::memory_order_acquire);
    if (before == after)
    {
      return true;
    }
  }
  return false;
}

int CrsfReceiver::read_byte()
{
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF);
    restart_dma();
  }

  const uint32_t write_index =
    (DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % DMA_BUFFER_SIZE;
  if (dma_read_index_ == write_index)
  {
    return -1;
  }

  const int byte = crsf_dma_buffer[dma_read_index_];
  dma_read_index_ = (dma_read_index_ + 1U) % DMA_BUFFER_SIZE;
  return byte;
}

void CrsfReceiver::restart_dma()
{
  (void)HAL_UART_DMAStop(huart_);
  dma_read_index_ = 0U;
  parser_.reset();
  std::memset(crsf_dma_buffer, 0, sizeof(crsf_dma_buffer));
  (void)HAL_UART_Receive_DMA(
    huart_, crsf_dma_buffer, static_cast<uint16_t>(DMA_BUFFER_SIZE));
}

void CrsfReceiver::update_snapshot(crsf::ParseResult result, uint32_t now_ms)
{
  snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_.valid_frame_count = parser_.valid_frame_count();
  snapshot_.crc_error_count = parser_.crc_error_count();

  if (result == crsf::ParseResult::RcChannels)
  {
    const crsf::RcChannels& channels = parser_.channels();
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
}

void CrsfReceiver::set_connected(bool connected)
{
  snapshot_version_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_.connected = connected;
  snapshot_version_.fetch_add(1U, std::memory_order_release);
}
