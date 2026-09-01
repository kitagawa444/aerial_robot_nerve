#include "rc/crsf_stm32_uart_transport.h"

#include <cstring>

namespace
{
#ifdef STM32H7
uint8_t crsf_dma_buffer[CrsfStm32UartTransport::DMA_BUFFER_SIZE] __attribute__((section(".CrsfRxBufferSection")));
#else
uint8_t crsf_dma_buffer[CrsfStm32UartTransport::DMA_BUFFER_SIZE];
#endif
}

bool CrsfStm32UartTransport::init(UART_HandleTypeDef *huart)
{
  if (huart == nullptr || huart->hdmarx == nullptr) return false;
  huart_ = huart;
  return true;
}

void CrsfStm32UartTransport::setEnabled(bool enabled)
{
  if (enabled_ == enabled) return;
  enabled_ = enabled;
  if (!enabled_)
  {
    if (huart_ != nullptr) (void)HAL_UART_DMAStop(huart_);
    dma_read_index_ = 0U;
    return;
  }
  (void)startDma_();
}

void CrsfStm32UartTransport::update(uint32_t now_ms)
{
  (void)now_ms;
  if (!enabled_ || huart_ == nullptr || huart_->hdmarx == nullptr) return;
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF);
    restartDma_();
  }
}

std::size_t CrsfStm32UartTransport::read(uint8_t *data, std::size_t capacity)
{
  if (!enabled_ || data == nullptr || capacity == 0U || huart_ == nullptr || huart_->hdmarx == nullptr) return 0U;

  const uint32_t write_index = (DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % DMA_BUFFER_SIZE;
  std::size_t count = 0U;
  while (dma_read_index_ != write_index && count < capacity)
  {
    data[count++] = crsf_dma_buffer[dma_read_index_];
    dma_read_index_ = (dma_read_index_ + 1U) % DMA_BUFFER_SIZE;
  }
  return count;
}

bool CrsfStm32UartTransport::startDma_()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr) return false;
  dma_read_index_ = 0U;
  std::memset(crsf_dma_buffer, 0, sizeof(crsf_dma_buffer));
  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  return HAL_UART_Receive_DMA(huart_, crsf_dma_buffer, static_cast<uint16_t>(DMA_BUFFER_SIZE)) == HAL_OK;
}

void CrsfStm32UartTransport::restartDma_()
{
  if (huart_ == nullptr) return;
  (void)HAL_UART_DMAStop(huart_);
  (void)startDma_();
}
