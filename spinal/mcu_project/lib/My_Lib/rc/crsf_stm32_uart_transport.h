#pragma once

#include "config.h"
#include "rc/crsf_transport.h"

#include <cstdint>

class CrsfStm32UartTransport final : public CrsfTransport
{
public:
  static constexpr uint32_t DMA_BUFFER_SIZE = 256U;

  bool init(UART_HandleTypeDef *huart);
  void setEnabled(bool enabled) override;
  void update(uint32_t now_ms) override;
  std::size_t read(uint8_t *data, std::size_t capacity) override;

private:
  UART_HandleTypeDef *huart_{ nullptr };
  uint32_t dma_read_index_{ 0U };
  bool enabled_{ false };

  bool startDma_();
  void restartDma_();
};
