#include "xrce_uart_transport.h"

#include <algorithm>

namespace {
XrceUartTransportContext *context(uxrCustomTransport *transport) {
  return transport == nullptr
             ? nullptr
             : static_cast<XrceUartTransportContext *>(transport->args);
}
} // namespace

bool xrce_uart_open(uxrCustomTransport *transport) {
  XrceUartTransportContext *ctx = context(transport);
  if (ctx == nullptr || ctx->uart == nullptr) {
    return false;
  }

  ctx->hardware.init(ctx->uart, ctx->tx_mutex, ctx->tx_semaphore);
  return true;
}

bool xrce_uart_close(uxrCustomTransport *transport) {
  XrceUartTransportContext *ctx = context(transport);
  if (ctx == nullptr) {
    return false;
  }

  ctx->hardware.deinit();
  return true;
}

size_t xrce_uart_write(uxrCustomTransport *transport, const uint8_t *buffer,
                       size_t length, uint8_t *error_code) {
  XrceUartTransportContext *ctx = context(transport);
  if (ctx == nullptr || buffer == nullptr || length == 0U ||
      length > UINT16_MAX) {
    if (error_code != nullptr)
      *error_code = 1U;
    return 0U;
  }

  ctx->hardware.write(const_cast<uint8_t *>(buffer),
                      static_cast<uint16_t>(length));
  while (true) {
    const int result = ctx->hardware.publish();
    if (result == BUFFER_EMPTY)
      break;
    if (result != HAL_OK) {
      if (error_code != nullptr)
        *error_code = 2U;
      return 0U;
    }
    osThreadYield();
  }

  if (error_code != nullptr)
    *error_code = 0U;
  return length;
}

size_t xrce_uart_read(uxrCustomTransport *transport, uint8_t *buffer,
                      size_t length, int timeout_ms, uint8_t *error_code) {
  XrceUartTransportContext *ctx = context(transport);
  if (ctx == nullptr || buffer == nullptr || length == 0U) {
    if (error_code != nullptr)
      *error_code = 1U;
    return 0U;
  }

  const uint32_t start_ms = HAL_GetTick();
  size_t received = 0U;
  while (received < length) {
    const int value = ctx->hardware.read();
    if (value >= 0) {
      buffer[received++] = static_cast<uint8_t>(value);
      continue;
    }

    if (timeout_ms == 0 ||
        (timeout_ms > 0 &&
         HAL_GetTick() - start_ms >= static_cast<uint32_t>(timeout_ms))) {
      break;
    }
    osDelay(1U);
  }

  if (error_code != nullptr)
    *error_code = 0U;
  return received;
}
