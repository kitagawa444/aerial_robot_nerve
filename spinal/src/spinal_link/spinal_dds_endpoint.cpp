#include "spinal_dds_endpoint.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastrtps/types/DynamicData.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicPubSubType.h>
#include <fastrtps/types/DynamicTypeBuilder.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>

namespace aerial::vehicle {
namespace {
using eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
using eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
using eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
using eprosima::fastdds::dds::DomainParticipant;
using eprosima::fastdds::dds::DomainParticipantFactory;
using eprosima::fastdds::dds::DomainParticipantQos;
using eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
using eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
using eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
using eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
using eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
using eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
using eprosima::fastdds::dds::TypeSupport;
using eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
using eprosima::fastrtps::types::DynamicData;
using eprosima::fastrtps::types::DynamicDataFactory;
using eprosima::fastrtps::types::DynamicPubSubType;
using eprosima::fastrtps::types::DynamicType_ptr;
using eprosima::fastrtps::types::DynamicTypeBuilderFactory;
using eprosima::fastrtps::types::MemberId;

constexpr char kDdsTypeName[] = "aerial::spinal_link::SpinalLinkFrame";
constexpr MemberId kProtocolVersionMember = 0U;
constexpr MemberId kMessageIdMember = 1U;
constexpr MemberId kSequenceMember = 2U;
constexpr MemberId kRequestIdMember = 3U;
constexpr MemberId kTimestampMember = 4U;
constexpr MemberId kSourceMember = 5U;
constexpr MemberId kReliabilityMember = 6U;
constexpr MemberId kPayloadMember = 7U;

bool ok(const eprosima::fastrtps::types::ReturnCode_t &result) {
  return result == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK;
}

uint64_t monotonicMillis() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

DynamicType_ptr createSpinalLinkFrameType() {
  auto *factory = DynamicTypeBuilderFactory::get_instance();
  auto *builder = factory->create_struct_builder();
  auto *uint16_builder = factory->create_uint16_builder();
  auto *uint32_builder = factory->create_uint32_builder();
  auto *uint64_builder = factory->create_uint64_builder();
  auto *byte_builder = factory->create_byte_builder();
  auto *payload_builder =
      factory->create_sequence_builder(byte_builder, kMaxPayloadSize);

  bool valid = builder != nullptr && uint16_builder != nullptr &&
               uint32_builder != nullptr && uint64_builder != nullptr &&
               byte_builder != nullptr && payload_builder != nullptr;
  if (valid) {
    valid =
        ok(builder->set_name(kDdsTypeName)) &&
        ok(builder->add_member(kProtocolVersionMember, "protocol_version",
                               uint16_builder)) &&
        ok(builder->add_member(kMessageIdMember, "message_id",
                               uint16_builder)) &&
        ok(builder->add_member(kSequenceMember, "sequence", uint32_builder)) &&
        ok(builder->add_member(kRequestIdMember, "request_id",
                               uint32_builder)) &&
        ok(builder->add_member(kTimestampMember, "timestamp_us",
                               uint64_builder)) &&
        ok(builder->add_member(kSourceMember, "source", byte_builder)) &&
        ok(builder->add_member(kReliabilityMember, "reliability",
                               byte_builder)) &&
        ok(builder->add_member(kPayloadMember, "payload", payload_builder));
  }

  DynamicType_ptr type = valid ? builder->build() : DynamicType_ptr{};
  if (payload_builder != nullptr)
    factory->delete_builder(payload_builder);
  if (byte_builder != nullptr)
    factory->delete_builder(byte_builder);
  if (uint64_builder != nullptr)
    factory->delete_builder(uint64_builder);
  if (uint32_builder != nullptr)
    factory->delete_builder(uint32_builder);
  if (uint16_builder != nullptr)
    factory->delete_builder(uint16_builder);
  if (builder != nullptr)
    factory->delete_builder(builder);
  return type;
}

bool copyToDynamicData(const Frame &frame, DynamicData &data) {
  if (!validFrame(frame) || !ok(data.clear_all_values()) ||
      !ok(data.set_uint16_value(frame.header.protocol_version,
                                kProtocolVersionMember)) ||
      !ok(data.set_uint16_value(frame.header.message_id, kMessageIdMember)) ||
      !ok(data.set_uint32_value(frame.header.sequence, kSequenceMember)) ||
      !ok(data.set_uint32_value(frame.header.request_id, kRequestIdMember)) ||
      !ok(data.set_uint64_value(frame.header.timestamp_us, kTimestampMember)) ||
      !ok(data.set_uint8_value(frame.header.source, kSourceMember)) ||
      !ok(data.set_uint8_value(frame.header.reliability, kReliabilityMember))) {
    return false;
  }

  DynamicData *payload = data.loan_value(kPayloadMember);
  if (payload == nullptr)
    return false;
  while (payload->get_item_count() > 0U) {
    const MemberId last =
        payload->get_member_id_at_index(payload->get_item_count() - 1U);
    if (!ok(payload->remove_sequence_data(last))) {
      (void)data.return_loaned_value(payload);
      return false;
    }
  }
  bool success = true;
  for (uint16_t index = 0U; index < frame.header.payload_size; ++index) {
    MemberId member_id{};
    if (!ok(payload->insert_byte_value(frame.payload[index], member_id))) {
      success = false;
      break;
    }
  }
  return ok(data.return_loaned_value(payload)) && success;
}

bool copyFromDynamicData(const DynamicData &data, Frame &frame) {
  frame = Frame{};
  if (!ok(data.get_uint16_value(frame.header.protocol_version,
                                kProtocolVersionMember)) ||
      !ok(data.get_uint16_value(frame.header.message_id, kMessageIdMember)) ||
      !ok(data.get_uint32_value(frame.header.sequence, kSequenceMember)) ||
      !ok(data.get_uint32_value(frame.header.request_id, kRequestIdMember)) ||
      !ok(data.get_uint64_value(frame.header.timestamp_us, kTimestampMember)) ||
      !ok(data.get_uint8_value(frame.header.source, kSourceMember)) ||
      !ok(data.get_uint8_value(frame.header.reliability, kReliabilityMember))) {
    return false;
  }

  auto &mutable_data = const_cast<DynamicData &>(data);
  DynamicData *payload = mutable_data.loan_value(kPayloadMember);
  if (payload == nullptr)
    return false;
  const uint32_t payload_size =
      std::min<uint32_t>(payload->get_item_count(), kMaxPayloadSize);
  bool success = payload->get_item_count() <= kMaxPayloadSize;
  for (uint32_t index = 0U; index < payload_size && success; ++index)
    success = ok(payload->get_byte_value(frame.payload[index], index));
  success = ok(mutable_data.return_loaned_value(payload)) && success;
  frame.header.payload_size = static_cast<uint16_t>(payload_size);
  return success && validFrame(frame);
}

bool publishesDirection(SpinalDdsEndpoint::Role role,
                        TopicDirection direction) {
  return role == SpinalDdsEndpoint::Role::HOST
             ? direction == TopicDirection::HOST_TO_SPINAL
             : direction == TopicDirection::SPINAL_TO_HOST;
}
} // namespace

struct SpinalDdsEndpoint::Implementation {
  struct Entity {
    const TopicDescriptor *descriptor{nullptr};
    eprosima::fastdds::dds::Topic *topic{nullptr};
    eprosima::fastdds::dds::DataWriter *writer{nullptr};
    eprosima::fastdds::dds::DataReader *reader{nullptr};
  };

  struct PendingRequest {
    Frame frame{};
    std::chrono::steady_clock::time_point created{};
    std::chrono::steady_clock::time_point deadline{};
    uint8_t retries{0U};
  };

  struct ReaderListener : public eprosima::fastdds::dds::DataReaderListener {
    explicit ReaderListener(Implementation &owner) : owner_(owner) {}

    void
    on_data_available(eprosima::fastdds::dds::DataReader *reader) override {
      const TopicDescriptor *descriptor = nullptr;
      for (const auto &entity : owner_.entities_) {
        if (entity.reader == reader) {
          descriptor = entity.descriptor;
          break;
        }
      }
      if (descriptor == nullptr)
        return;
      while (true) {
        DynamicData *data = DynamicDataFactory::get_instance()->create_data(
            owner_.dynamic_type_);
        if (data == nullptr)
          return;
        eprosima::fastdds::dds::SampleInfo info;
        const auto result = reader->take_next_sample(data, &info);
        if (result != eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
          DynamicDataFactory::get_instance()->delete_data(data);
          break;
        }
        Frame frame;
        const bool decoded = info.valid_data &&
                             copyFromDynamicData(*data, frame) &&
                             frame.header.message_id ==
                                 static_cast<uint16_t>(descriptor->message_id);
        DynamicDataFactory::get_instance()->delete_data(data);
        if (!decoded) {
          if (info.valid_data) {
            std::lock_guard<std::mutex> lock(owner_.statistics_mutex_);
            ++owner_.statistics_.receive_errors;
          }
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(owner_.statistics_mutex_);
          ++owner_.statistics_.received_frames;
          owner_.statistics_.last_receive_monotonic_ms = monotonicMillis();
        }
        owner_.handleAcknowledgement(frame);
        FrameCallback callback;
        {
          std::lock_guard<std::mutex> lock(owner_.callback_mutex_);
          callback = owner_.callback_;
        }
        if (callback)
          callback(frame);
      }
    }

    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader *,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus &status)
        override {
      std::lock_guard<std::mutex> lock(owner_.statistics_mutex_);
      const int64_t matched =
          static_cast<int64_t>(owner_.statistics_.matched_publications) +
          status.current_count_change;
      owner_.statistics_.matched_publications =
          static_cast<uint32_t>(std::max<int64_t>(0, matched));
    }

    Implementation &owner_;
  };

  struct WriterListener : public eprosima::fastdds::dds::DataWriterListener {
    explicit WriterListener(Implementation &owner) : owner_(owner) {}

    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter *,
        const eprosima::fastdds::dds::PublicationMatchedStatus &status)
        override {
      std::lock_guard<std::mutex> lock(owner_.statistics_mutex_);
      const int64_t matched =
          static_cast<int64_t>(owner_.statistics_.matched_subscriptions) +
          status.current_count_change;
      owner_.statistics_.matched_subscriptions =
          static_cast<uint32_t>(std::max<int64_t>(0, matched));
    }

    Implementation &owner_;
  };

  Implementation() : listener_(*this), writer_listener_(*this) {}

  Entity *entityFor(MessageId message_id) {
    for (auto &entity : entities_) {
      if (entity.descriptor->message_id == message_id)
        return &entity;
    }
    return nullptr;
  }

  bool writeFrame(const Frame &frame) {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    Entity *entity = entityFor(static_cast<MessageId>(frame.header.message_id));
    if (!running_.load() || entity == nullptr || entity->writer == nullptr) {
      std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
      ++statistics_.transmit_errors;
      return false;
    }
    DynamicData *data =
        DynamicDataFactory::get_instance()->create_data(dynamic_type_);
    if (data == nullptr) {
      std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
      ++statistics_.transmit_errors;
      return false;
    }
    const bool success =
        copyToDynamicData(frame, *data) && entity->writer->write(data);
    DynamicDataFactory::get_instance()->delete_data(data);
    {
      std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
      if (success) {
        ++statistics_.transmitted_frames;
        statistics_.last_transmit_monotonic_ms = monotonicMillis();
      } else {
        ++statistics_.transmit_errors;
      }
    }
    return success;
  }

  void handleAcknowledgement(const Frame &frame) {
    if (frame.header.message_id !=
        static_cast<uint16_t>(MessageId::PROTOCOL_ACK))
      return;
    ProtocolAck ack{};
    if (!getPayload(frame, MessageId::PROTOCOL_ACK, ack))
      return;
    const auto now = std::chrono::steady_clock::now();
    bool matched = false;
    uint32_t rtt_ms = 0U;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_.erase(
          std::remove_if(
              pending_.begin(), pending_.end(),
              [&ack, &now, &matched, &rtt_ms](const PendingRequest &pending) {
                const bool same =
                    pending.frame.header.request_id == ack.request_id &&
                    pending.frame.header.message_id == ack.message_id &&
                    pending.frame.header.source == ack.request_source;
                if (same) {
                  matched = true;
                  rtt_ms = static_cast<uint32_t>(
                      std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - pending.created)
                          .count());
                }
                return same;
              }),
          pending_.end());
    }
    // Every endpoint on the DDS domain can observe protocol ACKs. Only account
    // for an ACK when it completes a request owned by this endpoint.
    if (!matched)
      return;
    std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
    ++statistics_.ack_received;
    statistics_.last_ack_rtt_ms = rtt_ms;
    statistics_.last_ack_message_id = ack.message_id;
    statistics_.last_ack_request_id = ack.request_id;
    if (ack.duplicate != 0U)
      ++statistics_.duplicate_acks;
    if (ack.result != static_cast<uint8_t>(ProtocolAckResult::ACCEPTED))
      ++statistics_.rejected_acks;
  }

  void retryLoop() {
    while (running_.load()) {
      std::vector<Frame> retries;
      const auto now = std::chrono::steady_clock::now();
      {
        std::unique_lock<std::mutex> lock(pending_mutex_);
        pending_condition_.wait_for(lock, std::chrono::milliseconds(10));
        for (auto iterator = pending_.begin(); iterator != pending_.end();) {
          if (iterator->deadline > now) {
            ++iterator;
            continue;
          }
          const auto *descriptor = enabledTopicDescriptor(
              static_cast<MessageId>(iterator->frame.header.message_id));
          if (descriptor == nullptr ||
              iterator->retries >= descriptor->max_retries) {
            {
              std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
              ++statistics_.ack_timeouts;
              statistics_.last_ack_timeout_monotonic_ms = monotonicMillis();
              statistics_.last_ack_message_id =
                  iterator->frame.header.message_id;
              statistics_.last_ack_request_id =
                  iterator->frame.header.request_id;
            }
            iterator = pending_.erase(iterator);
            continue;
          }
          retries.push_back(iterator->frame);
          ++iterator->retries;
          {
            std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
            ++statistics_.ack_retries;
          }
          iterator->deadline =
              now + std::chrono::milliseconds(descriptor->ack_timeout_ms);
          ++iterator;
        }
      }
      for (const Frame &frame : retries)
        (void)writeFrame(frame);
    }
  }

  DomainParticipant *participant_{nullptr};
  eprosima::fastdds::dds::Publisher *publisher_{nullptr};
  eprosima::fastdds::dds::Subscriber *subscriber_{nullptr};
  std::vector<Entity> entities_;
  TypeSupport type_;
  DynamicType_ptr dynamic_type_;
  ReaderListener listener_;
  WriterListener writer_listener_;
  FrameCallback callback_;
  std::mutex callback_mutex_;
  std::mutex writer_mutex_;
  mutable std::mutex pending_mutex_;
  mutable std::mutex statistics_mutex_;
  std::condition_variable pending_condition_;
  std::vector<PendingRequest> pending_;
  std::thread retry_thread_;
  std::atomic<uint32_t> request_sequence_{0U};
  std::atomic<bool> running_{false};
  SpinalDdsStatistics statistics_{};
  Role role_{Role::HOST};
};

SpinalDdsEndpoint::SpinalDdsEndpoint() : implementation_(new Implementation) {}

SpinalDdsEndpoint::~SpinalDdsEndpoint() { stop(); }

bool SpinalDdsEndpoint::start(const std::string &participant_name,
                              uint32_t domain_id, Role role) {
  stop();
  auto &impl = *implementation_;
  impl.role_ = role;
  {
    std::lock_guard<std::mutex> lock(impl.statistics_mutex_);
    impl.statistics_ = SpinalDdsStatistics{};
    impl.statistics_.started_monotonic_ms = monotonicMillis();
  }
  impl.dynamic_type_ = createSpinalLinkFrameType();
  if (!impl.dynamic_type_)
    return false;
  impl.type_ = TypeSupport(new DynamicPubSubType(impl.dynamic_type_));
  impl.type_.get()->auto_fill_type_information(false);
  impl.type_.get()->auto_fill_type_object(false);

  DomainParticipantQos participant_qos;
  participant_qos.name(participant_name);
  impl.participant_ =
      DomainParticipantFactory::get_instance()->create_participant(
          domain_id, participant_qos);
  if (impl.participant_ == nullptr ||
      !impl.type_.register_type(impl.participant_)) {
    stop();
    return false;
  }
  impl.publisher_ =
      impl.participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  impl.subscriber_ =
      impl.participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
  if (impl.publisher_ == nullptr || impl.subscriber_ == nullptr) {
    stop();
    return false;
  }

  for (const auto &descriptor : kSpinalLinkTopics) {
    if (!descriptor.enabled)
      continue;
    Implementation::Entity entity;
    entity.descriptor = &descriptor;
    entity.topic = impl.participant_->create_topic(
        descriptor.topic_name, kDdsTypeName, TOPIC_QOS_DEFAULT);
    if (entity.topic == nullptr) {
      stop();
      return false;
    }
    if (publishesDirection(role, descriptor.direction)) {
      auto qos = DATAWRITER_QOS_DEFAULT;
      qos.reliability().kind = descriptor.reliability == Reliability::RELIABLE
                                   ? RELIABLE_RELIABILITY_QOS
                                   : BEST_EFFORT_RELIABILITY_QOS;
      qos.durability().kind =
          descriptor.durability == TopicDurability::TRANSIENT_LOCAL
              ? TRANSIENT_LOCAL_DURABILITY_QOS
              : VOLATILE_DURABILITY_QOS;
      qos.history().kind = KEEP_LAST_HISTORY_QOS;
      qos.history().depth = descriptor.history_depth;
      entity.writer = impl.publisher_->create_datawriter(
          entity.topic, qos, &impl.writer_listener_);
      if (entity.writer == nullptr) {
        impl.entities_.push_back(entity);
        stop();
        return false;
      }
    } else {
      auto qos = DATAREADER_QOS_DEFAULT;
      qos.reliability().kind = descriptor.reliability == Reliability::RELIABLE
                                   ? RELIABLE_RELIABILITY_QOS
                                   : BEST_EFFORT_RELIABILITY_QOS;
      qos.durability().kind =
          descriptor.durability == TopicDurability::TRANSIENT_LOCAL
              ? TRANSIENT_LOCAL_DURABILITY_QOS
              : VOLATILE_DURABILITY_QOS;
      qos.history().kind = KEEP_LAST_HISTORY_QOS;
      qos.history().depth = descriptor.history_depth;
      entity.reader = impl.subscriber_->create_datareader(entity.topic, qos,
                                                          &impl.listener_);
      if (entity.reader == nullptr) {
        impl.entities_.push_back(entity);
        stop();
        return false;
      }
    }
    impl.entities_.push_back(entity);
  }
  impl.running_.store(true);
  {
    std::lock_guard<std::mutex> lock(impl.statistics_mutex_);
    impl.statistics_.running = true;
  }
  if (role == Role::HOST)
    impl.retry_thread_ = std::thread([&impl]() { impl.retryLoop(); });
  return true;
}

void SpinalDdsEndpoint::stop() {
  auto &impl = *implementation_;
  impl.running_.store(false);
  {
    std::lock_guard<std::mutex> lock(impl.statistics_mutex_);
    impl.statistics_.running = false;
  }
  impl.pending_condition_.notify_all();
  if (impl.retry_thread_.joinable())
    impl.retry_thread_.join();
  {
    std::lock_guard<std::mutex> lock(impl.pending_mutex_);
    impl.pending_.clear();
  }
  for (auto &entity : impl.entities_) {
    if (impl.subscriber_ != nullptr && entity.reader != nullptr)
      impl.subscriber_->delete_datareader(entity.reader);
    if (impl.publisher_ != nullptr && entity.writer != nullptr)
      impl.publisher_->delete_datawriter(entity.writer);
  }
  for (auto &entity : impl.entities_) {
    if (impl.participant_ != nullptr && entity.topic != nullptr)
      impl.participant_->delete_topic(entity.topic);
  }
  impl.entities_.clear();
  if (impl.participant_ != nullptr && impl.subscriber_ != nullptr)
    impl.participant_->delete_subscriber(impl.subscriber_);
  if (impl.participant_ != nullptr && impl.publisher_ != nullptr)
    impl.participant_->delete_publisher(impl.publisher_);
  impl.subscriber_ = nullptr;
  impl.publisher_ = nullptr;
  if (impl.participant_ != nullptr)
    DomainParticipantFactory::get_instance()->delete_participant(
        impl.participant_);
  impl.participant_ = nullptr;
  impl.type_.reset();
  impl.dynamic_type_.reset();
}

bool SpinalDdsEndpoint::publish(const Frame &input) {
  auto &impl = *implementation_;
  Frame frame = input;
  const auto *descriptor =
      enabledTopicDescriptor(static_cast<MessageId>(frame.header.message_id));
  if (descriptor == nullptr ||
      !publishesDirection(impl.role_, descriptor->direction))
    return false;
  frame.header.reliability = static_cast<uint8_t>(descriptor->reliability);
  if (impl.role_ == Role::HOST && descriptor->application_ack &&
      frame.header.request_id == 0U) {
    uint32_t request_id = ++impl.request_sequence_;
    if (request_id == 0U)
      request_id = ++impl.request_sequence_;
    frame.header.request_id = request_id;
  }
  if (impl.role_ == Role::HOST && descriptor->application_ack) {
    Implementation::PendingRequest pending;
    pending.frame = frame;
    pending.created = std::chrono::steady_clock::now();
    pending.deadline =
        pending.created + std::chrono::milliseconds(descriptor->ack_timeout_ms);
    std::lock_guard<std::mutex> lock(impl.pending_mutex_);
    impl.pending_.erase(
        std::remove_if(impl.pending_.begin(), impl.pending_.end(),
                       [&frame](const Implementation::PendingRequest &item) {
                         return item.frame.header.request_id ==
                                    frame.header.request_id &&
                                item.frame.header.message_id ==
                                    frame.header.message_id &&
                                item.frame.header.source == frame.header.source;
                       }),
        impl.pending_.end());
    impl.pending_.push_back(pending);
    impl.pending_condition_.notify_all();
  }
  if (impl.writeFrame(frame))
    return true;
  if (impl.role_ == Role::HOST && descriptor->application_ack) {
    std::lock_guard<std::mutex> lock(impl.pending_mutex_);
    impl.pending_.erase(
        std::remove_if(impl.pending_.begin(), impl.pending_.end(),
                       [&frame](const Implementation::PendingRequest &item) {
                         return item.frame.header.request_id ==
                                    frame.header.request_id &&
                                item.frame.header.message_id ==
                                    frame.header.message_id &&
                                item.frame.header.source == frame.header.source;
                       }),
        impl.pending_.end());
  }
  return false;
}

void SpinalDdsEndpoint::setFrameCallback(FrameCallback callback) {
  std::lock_guard<std::mutex> lock(implementation_->callback_mutex_);
  implementation_->callback_ = std::move(callback);
}

bool SpinalDdsEndpoint::running() const {
  return implementation_->running_.load();
}

size_t SpinalDdsEndpoint::pendingRequestCount() const {
  std::lock_guard<std::mutex> lock(implementation_->pending_mutex_);
  return implementation_->pending_.size();
}

SpinalDdsStatistics SpinalDdsEndpoint::statistics() const {
  auto &impl = *implementation_;
  SpinalDdsStatistics result;
  {
    std::lock_guard<std::mutex> lock(impl.statistics_mutex_);
    result = impl.statistics_;
  }
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(impl.pending_mutex_);
    result.pending_requests = static_cast<uint32_t>(impl.pending_.size());
    if (!impl.pending_.empty()) {
      const auto oldest =
          std::min_element(impl.pending_.begin(), impl.pending_.end(),
                           [](const Implementation::PendingRequest &left,
                              const Implementation::PendingRequest &right) {
                             return left.created < right.created;
                           });
      result.oldest_pending_age_ms = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                oldest->created)
              .count());
    }
  }
  result.running = impl.running_.load();
  return result;
}

} // namespace aerial::vehicle
