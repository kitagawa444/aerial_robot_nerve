#pragma once

#include <cstddef>
#include <cstdint>

#include <ucdr/microcdr.h>
#include <uxr/client/client.h>

#include "communication/spinal_link_protocol.h"
#include "communication/spinal_link_topics.generated.h"
#include "xrce_uart_transport.h"

class XrceDdsClient {
public:
  using FrameCallback = void (*)(const aerial::vehicle::Frame &frame,
                                 void *argument);

  XrceDdsClient();

  void configure(UART_HandleTypeDef *uart, osMutexId *tx_mutex,
                 osSemaphoreId *tx_semaphore,
                 uint32_t client_key = 0x41525631U);
  bool connect();
  void disconnect();
  bool connected() const { return connected_; }
  bool spin(uint32_t timeout_ms);
  bool publish(const aerial::vehicle::Frame &frame);
  void setFrameCallback(FrameCallback callback, void *argument);

private:
  static constexpr size_t kStreamHistory = 4U;
  static constexpr size_t kStreamBufferSize =
      UXR_CONFIG_CUSTOM_TRANSPORT_MTU * kStreamHistory;

  XrceUartTransportContext transport_context_{};
  uxrCustomTransport transport_{};
  uxrSession session_{};
  uxrStreamId reliable_output_{};
  uxrStreamId reliable_input_{};
  uxrStreamId best_effort_output_{};
  uxrStreamId best_effort_input_{};
  uxrObjectId writers_[aerial::vehicle::kSpinalLinkTopicCount]{};
  uxrObjectId readers_[aerial::vehicle::kSpinalLinkTopicCount]{};
  uint8_t reliable_output_buffer_[kStreamBufferSize]{};
  uint8_t reliable_input_buffer_[kStreamBufferSize]{};
  uint8_t best_effort_output_buffer_[UXR_CONFIG_CUSTOM_TRANSPORT_MTU]{};
  uint32_t client_key_{0x41525631U};
  bool configured_{false};
  bool session_created_{false};
  bool connected_{false};
  FrameCallback frame_callback_{nullptr};
  void *frame_callback_argument_{nullptr};

  static void topicCallback(uxrSession *session, uxrObjectId object_id,
                            uint16_t request_id, uxrStreamId stream_id,
                            ucdrBuffer *buffer, uint16_t length,
                            void *argument);
  bool createEntities();
  bool createAndConfirm(uint16_t request_id);
  size_t
  descriptorIndex(const aerial::vehicle::TopicDescriptor *descriptor) const;
  bool serializeFrame(ucdrBuffer &buffer,
                      const aerial::vehicle::Frame &frame) const;
  bool deserializeFrame(ucdrBuffer &buffer,
                        aerial::vehicle::Frame &frame) const;
  size_t serializedSize(const aerial::vehicle::Frame &frame) const;
};
