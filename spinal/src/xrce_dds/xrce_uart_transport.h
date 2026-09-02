#pragma once

#include <cstddef>
#include <cstdint>

#include <uxr/client/profile/transport/custom/custom_transport.h>

#include "STM32Hardware.h"

struct XrceUartTransportContext {
  STM32Hardware hardware;
  UART_HandleTypeDef *uart{nullptr};
  osMutexId *tx_mutex{nullptr};
  osSemaphoreId *tx_semaphore{nullptr};
};

bool xrce_uart_open(uxrCustomTransport *transport);
bool xrce_uart_close(uxrCustomTransport *transport);
size_t xrce_uart_write(uxrCustomTransport *transport, const uint8_t *buffer,
                       size_t length, uint8_t *error_code);
size_t xrce_uart_read(uxrCustomTransport *transport, uint8_t *buffer,
                      size_t length, int timeout_ms, uint8_t *error_code);
