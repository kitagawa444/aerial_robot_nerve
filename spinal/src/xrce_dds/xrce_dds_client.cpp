#include "xrce_dds_client.h"

#include <cstdio>
#include <cstring>

namespace {
constexpr uint16_t kParticipantId = 0x01U;
constexpr uint16_t kPublisherId = 0x01U;
constexpr uint16_t kSubscriberId = 0x01U;

constexpr char kParticipantXml[] = "<dds><participant><rtps><name>spinal_xrce</"
                                   "name></rtps></participant></dds>";

const char *reliabilityName(aerial::vehicle::Reliability reliability) {
  return reliability == aerial::vehicle::Reliability::RELIABLE ? "RELIABLE"
                                                               : "BEST_EFFORT";
}

const char *durabilityName(aerial::vehicle::TopicDurability durability) {
  return durability == aerial::vehicle::TopicDurability::TRANSIENT_LOCAL
             ? "TRANSIENT_LOCAL"
             : "VOLATILE";
}
} // namespace

XrceDdsClient::XrceDdsClient() {
  for (size_t index = 0; index < aerial::vehicle::kSpinalLinkTopicCount;
       ++index) {
    const uint16_t id = aerial::vehicle::kSpinalLinkTopics[index].entity_id;
    writers_[index] = uxr_object_id(id, UXR_DATAWRITER_ID);
    readers_[index] = uxr_object_id(id, UXR_DATAREADER_ID);
  }
}

void XrceDdsClient::configure(UART_HandleTypeDef *uart, osMutexId *tx_mutex,
                              osSemaphoreId *tx_semaphore,
                              uint32_t client_key) {
  transport_context_.uart = uart;
  transport_context_.tx_mutex = tx_mutex;
  transport_context_.tx_semaphore = tx_semaphore;
  client_key_ = client_key;
  configured_ = uart != nullptr;
}

bool XrceDdsClient::connect() {
  if (!configured_)
    return false;
  disconnect();

  std::memset(&transport_, 0, sizeof(transport_));
  uxr_set_custom_transport_callbacks(&transport_, true, xrce_uart_open,
                                     xrce_uart_close, xrce_uart_write,
                                     xrce_uart_read);
  if (!uxr_init_custom_transport(&transport_, &transport_context_))
    return false;

  uxr_init_session(&session_, &transport_.comm, client_key_);
  uxr_set_topic_callback(&session_, &XrceDdsClient::topicCallback, this);
  if (!uxr_create_session(&session_)) {
    (void)uxr_close_custom_transport(&transport_);
    return false;
  }
  session_created_ = true;

  reliable_output_ = uxr_create_output_reliable_stream(
      &session_, reliable_output_buffer_, sizeof(reliable_output_buffer_),
      kStreamHistory);
  reliable_input_ = uxr_create_input_reliable_stream(
      &session_, reliable_input_buffer_, sizeof(reliable_input_buffer_),
      kStreamHistory);
  best_effort_output_ = uxr_create_output_best_effort_stream(
      &session_, best_effort_output_buffer_,
      sizeof(best_effort_output_buffer_));
  best_effort_input_ = uxr_create_input_best_effort_stream(&session_);

  connected_ = createEntities();
  if (!connected_)
    disconnect();
  return connected_;
}

void XrceDdsClient::disconnect() {
  if (session_created_)
    uxr_delete_session(&session_);
  if (transport_.comm.instance != nullptr)
    (void)uxr_close_custom_transport(&transport_);
  session_created_ = false;
  connected_ = false;
}

bool XrceDdsClient::createAndConfirm(uint16_t request_id) {
  uint8_t status = 0xFFU;
  if (!uxr_run_session_until_all_status(&session_, 2000, &request_id, &status,
                                        1U))
    return false;
  return status == UXR_STATUS_OK || status == UXR_STATUS_OK_MATCHED;
}

bool XrceDdsClient::createEntities() {
  const uxrObjectId participant_id =
      uxr_object_id(kParticipantId, UXR_PARTICIPANT_ID);
  const uxrObjectId publisher_id =
      uxr_object_id(kPublisherId, UXR_PUBLISHER_ID);
  const uxrObjectId subscriber_id =
      uxr_object_id(kSubscriberId, UXR_SUBSCRIBER_ID);

  if (!createAndConfirm(uxr_buffer_create_participant_xml(
          &session_, reliable_output_, participant_id, 0U, kParticipantXml,
          UXR_REPLACE)) ||
      !createAndConfirm(uxr_buffer_create_publisher_xml(
          &session_, reliable_output_, publisher_id, participant_id, "",
          UXR_REPLACE)) ||
      !createAndConfirm(uxr_buffer_create_subscriber_xml(
          &session_, reliable_output_, subscriber_id, participant_id, "",
          UXR_REPLACE))) {
    return false;
  }

  char topic_xml[320]{};
  char endpoint_xml[512]{};
  for (size_t index = 0; index < aerial::vehicle::kSpinalLinkTopicCount;
       ++index) {
    const auto &descriptor = aerial::vehicle::kSpinalLinkTopics[index];
    if (!descriptor.enabled)
      continue;
    const uxrObjectId topic_id =
        uxr_object_id(descriptor.entity_id, UXR_TOPIC_ID);
    const int topic_size = std::snprintf(
        topic_xml, sizeof(topic_xml),
        "<dds><topic><name>%s</name><dataType>aerial::spinal_link::"
        "SpinalLinkFrame</dataType></topic></dds>",
        descriptor.topic_name);
    if (topic_size <= 0 ||
        static_cast<size_t>(topic_size) >= sizeof(topic_xml) ||
        !createAndConfirm(uxr_buffer_create_topic_xml(
            &session_, reliable_output_, topic_id, participant_id, topic_xml,
            UXR_REPLACE))) {
      return false;
    }

    const bool writer =
        descriptor.direction == aerial::vehicle::TopicDirection::SPINAL_TO_HOST;
    const int endpoint_size = std::snprintf(
        endpoint_xml, sizeof(endpoint_xml),
        "<dds><%s><topic><kind>NO_KEY</kind><name>%s</name><dataType>"
        "aerial::spinal_link::SpinalLinkFrame</dataType></topic><qos>"
        "<reliability><kind>%s</kind></reliability><durability><kind>%s</kind>"
        "</durability><history><kind>KEEP_LAST</kind><depth>%u</depth>"
        "</history></qos></%s></dds>",
        writer ? "data_writer" : "data_reader", descriptor.topic_name,
        reliabilityName(descriptor.reliability),
        durabilityName(descriptor.durability), descriptor.history_depth,
        writer ? "data_writer" : "data_reader");
    if (endpoint_size <= 0 ||
        static_cast<size_t>(endpoint_size) >= sizeof(endpoint_xml)) {
      return false;
    }
    const uint16_t request =
        writer
            ? uxr_buffer_create_datawriter_xml(&session_, reliable_output_,
                                               writers_[index], publisher_id,
                                               endpoint_xml, UXR_REPLACE)
            : uxr_buffer_create_datareader_xml(&session_, reliable_output_,
                                               readers_[index], subscriber_id,
                                               endpoint_xml, UXR_REPLACE);
    if (!createAndConfirm(request))
      return false;
    if (!writer) {
      uxrDeliveryControl delivery{};
      delivery.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
      const uxrStreamId input =
          descriptor.reliability == aerial::vehicle::Reliability::RELIABLE
              ? reliable_input_
              : best_effort_input_;
      (void)uxr_buffer_request_data(&session_, reliable_output_,
                                    readers_[index], input, &delivery);
      if (!uxr_run_session_time(&session_, 100))
        return false;
    }
  }
  return true;
}

bool XrceDdsClient::spin(uint32_t timeout_ms) {
  if (!connected_)
    return false;
  connected_ = uxr_run_session_time(&session_, static_cast<int>(timeout_ms));
  return connected_;
}

size_t XrceDdsClient::descriptorIndex(
    const aerial::vehicle::TopicDescriptor *descriptor) const {
  return static_cast<size_t>(descriptor - aerial::vehicle::kSpinalLinkTopics);
}

bool XrceDdsClient::publish(const aerial::vehicle::Frame &frame) {
  if (!connected_ || !aerial::vehicle::validFrame(frame))
    return false;
  const auto *descriptor = aerial::vehicle::enabledTopicDescriptor(
      static_cast<aerial::vehicle::MessageId>(frame.header.message_id));
  if (descriptor == nullptr ||
      descriptor->direction != aerial::vehicle::TopicDirection::SPINAL_TO_HOST)
    return false;

  const size_t index = descriptorIndex(descriptor);
  const uxrStreamId output =
      descriptor->reliability == aerial::vehicle::Reliability::RELIABLE
          ? reliable_output_
          : best_effort_output_;
  ucdrBuffer buffer;
  const size_t size = serializedSize(frame);
  if (!uxr_prepare_output_stream(&session_, output, writers_[index], &buffer,
                                 size))
    return false;
  if (!serializeFrame(buffer, frame))
    return false;
  return uxr_run_session_time(&session_, 0);
}

void XrceDdsClient::setFrameCallback(FrameCallback callback, void *argument) {
  frame_callback_ = callback;
  frame_callback_argument_ = argument;
}

void XrceDdsClient::topicCallback(uxrSession *, uxrObjectId object_id, uint16_t,
                                  uxrStreamId, ucdrBuffer *buffer, uint16_t,
                                  void *argument) {
  auto *self = static_cast<XrceDdsClient *>(argument);
  if (self == nullptr || buffer == nullptr)
    return;

  const aerial::vehicle::TopicDescriptor *descriptor = nullptr;
  for (size_t index = 0; index < aerial::vehicle::kSpinalLinkTopicCount;
       ++index) {
    const auto &candidate = aerial::vehicle::kSpinalLinkTopics[index];
    if (candidate.enabled &&
        candidate.direction ==
            aerial::vehicle::TopicDirection::HOST_TO_SPINAL &&
        object_id.id == self->readers_[index].id) {
      descriptor = &candidate;
      break;
    }
  }
  if (descriptor == nullptr)
    return;

  aerial::vehicle::Frame frame;
  if (self->deserializeFrame(*buffer, frame) &&
      frame.header.message_id ==
          static_cast<uint16_t>(descriptor->message_id) &&
      self->frame_callback_ != nullptr) {
    self->frame_callback_(frame, self->frame_callback_argument_);
  }
}

bool XrceDdsClient::serializeFrame(ucdrBuffer &buffer,
                                   const aerial::vehicle::Frame &frame) const {
  return ucdr_serialize_uint16_t(&buffer, frame.header.protocol_version) &&
         ucdr_serialize_uint16_t(&buffer, frame.header.message_id) &&
         ucdr_serialize_uint32_t(&buffer, frame.header.sequence) &&
         ucdr_serialize_uint32_t(&buffer, frame.header.request_id) &&
         ucdr_serialize_uint64_t(&buffer, frame.header.timestamp_us) &&
         ucdr_serialize_uint8_t(&buffer, frame.header.source) &&
         ucdr_serialize_uint8_t(&buffer, frame.header.reliability) &&
         ucdr_serialize_sequence_uint8_t(&buffer, frame.payload,
                                         frame.header.payload_size);
}

bool XrceDdsClient::deserializeFrame(ucdrBuffer &buffer,
                                     aerial::vehicle::Frame &frame) const {
  uint32_t payload_size = 0U;
  if (!ucdr_deserialize_uint16_t(&buffer, &frame.header.protocol_version) ||
      !ucdr_deserialize_uint16_t(&buffer, &frame.header.message_id) ||
      !ucdr_deserialize_uint32_t(&buffer, &frame.header.sequence) ||
      !ucdr_deserialize_uint32_t(&buffer, &frame.header.request_id) ||
      !ucdr_deserialize_uint64_t(&buffer, &frame.header.timestamp_us) ||
      !ucdr_deserialize_uint8_t(&buffer, &frame.header.source) ||
      !ucdr_deserialize_uint8_t(&buffer, &frame.header.reliability) ||
      !ucdr_deserialize_sequence_uint8_t(&buffer, frame.payload,
                                         aerial::vehicle::kMaxPayloadSize,
                                         &payload_size)) {
    return false;
  }
  frame.header.payload_size = static_cast<uint16_t>(payload_size);
  return aerial::vehicle::validFrame(frame);
}

size_t
XrceDdsClient::serializedSize(const aerial::vehicle::Frame &frame) const {
  size_t size = 0U;
  size += ucdr_alignment(size, sizeof(uint16_t)) + sizeof(uint16_t);
  size += ucdr_alignment(size, sizeof(uint16_t)) + sizeof(uint16_t);
  size += ucdr_alignment(size, sizeof(uint32_t)) + sizeof(uint32_t);
  size += ucdr_alignment(size, sizeof(uint32_t)) + sizeof(uint32_t);
  size += ucdr_alignment(size, sizeof(uint64_t)) + sizeof(uint64_t);
  size += sizeof(uint8_t) * 2U;
  size += ucdr_alignment(size, sizeof(uint32_t)) + sizeof(uint32_t);
  size += frame.header.payload_size;
  return size;
}
