#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <unistd.h>

#include "communication/spinal_link_protocol.h"
#include "communication_monitor.h"
#include "spinal_dds_endpoint.h"

namespace {
using aerial::vehicle::Frame;
using aerial::vehicle::Heartbeat;
using aerial::vehicle::MessageId;
using aerial::vehicle::Reliability;
using aerial::vehicle::Source;
using aerial::vehicle::SpinalDdsEndpoint;

TEST(SpinalDdsEndpoint, ExchangesSpinalLinkFrameWithoutRos) {
  SpinalDdsEndpoint first;
  SpinalDdsEndpoint second;
  std::mutex mutex;
  std::condition_variable condition;
  Frame received;
  bool received_frame = false;
  std::atomic<bool> ack_publish_success{true};

  second.setFrameCallback([&](const Frame &frame) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      received = frame;
      received_frame = true;
    }
    condition.notify_one();
    const auto *descriptor = aerial::vehicle::enabledTopicDescriptor(
        static_cast<MessageId>(frame.header.message_id));
    if (descriptor != nullptr && descriptor->application_ack) {
      aerial::vehicle::ProtocolAck ack{};
      ack.request_id = frame.header.request_id;
      ack.message_id = frame.header.message_id;
      ack.request_source = frame.header.source;
      Frame ack_frame;
      const bool encoded = aerial::vehicle::setPayload(
          ack_frame, MessageId::PROTOCOL_ACK, ack, frame.header.sequence, 0U,
          Source::INTERNAL, Reliability::RELIABLE);
      if (!encoded || !second.publish(ack_frame))
        ack_publish_success.store(false);
    }
  });

  const uint32_t domain_id = 100U + static_cast<uint32_t>(getpid() % 100);
  ASSERT_TRUE(first.start("spinal_dds_test_first", domain_id));
  ASSERT_TRUE(second.start("spinal_dds_test_second", domain_id,
                           SpinalDdsEndpoint::Role::SPINAL_TEST));

  Heartbeat heartbeat{};
  heartbeat.uptime_ms = 123456U;
  heartbeat.link_state = 3U;
  Frame sent;
  ASSERT_TRUE(aerial::vehicle::setPayload(
      sent, MessageId::NETWORK_HEARTBEAT, heartbeat, 42U, 987654U,
      Source::MAVLINK, Reliability::BEST_EFFORT));

  bool published = false;
  for (size_t attempt = 0; attempt < 20U; ++attempt) {
    published = first.publish(sent) || published;
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (condition.wait_for(lock, std::chrono::milliseconds(100),
                             [&]() { return received_frame; }))
        break;
    }
  }
  ASSERT_TRUE(published);

  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(received_frame);
  EXPECT_EQ(received.header.protocol_version,
            aerial::vehicle::kProtocolVersion);
  EXPECT_EQ(received.header.sequence, 42U);
  EXPECT_EQ(received.header.timestamp_us, 987654U);
  EXPECT_EQ(received.header.source, static_cast<uint8_t>(Source::MAVLINK));
  Heartbeat decoded{};
  ASSERT_TRUE(aerial::vehicle::getPayload(
      received, MessageId::NETWORK_HEARTBEAT, decoded));
  EXPECT_EQ(decoded.uptime_ms, heartbeat.uptime_ms);
  EXPECT_EQ(decoded.link_state, heartbeat.link_state);

  lock.unlock();
  aerial::vehicle::ConfigFlashRequest config_request{};
  config_request.request_id = 7U;
  Frame larger_frame;
  ASSERT_TRUE(aerial::vehicle::setPayload(
      larger_frame, MessageId::CONFIG_FLASH_REQUEST, config_request, 43U,
      987655U, Source::ROS2, Reliability::RELIABLE));
  ASSERT_TRUE(first.publish(larger_frame));
  {
    std::unique_lock<std::mutex> wait_lock(mutex);
    ASSERT_TRUE(condition.wait_for(wait_lock, std::chrono::seconds(1), [&]() {
      return received.header.sequence == 43U;
    }));
    EXPECT_NE(received.header.request_id, 0U);
  }
  for (size_t attempt = 0U; attempt < 20U && first.pendingRequestCount() != 0U;
       ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  EXPECT_EQ(first.pendingRequestCount(), 0U);
  EXPECT_TRUE(ack_publish_success.load());

  aerial::vehicle::RebootRequest reboot_request{};
  reboot_request.request_id = 8U;
  Frame smaller_frame;
  ASSERT_TRUE(aerial::vehicle::setPayload(
      smaller_frame, MessageId::REBOOT_REQUEST, reboot_request, 44U, 987656U,
      Source::ROS2, Reliability::RELIABLE));
  ASSERT_TRUE(first.publish(smaller_frame));
  {
    std::unique_lock<std::mutex> wait_lock(mutex);
    ASSERT_TRUE(condition.wait_for(wait_lock, std::chrono::seconds(1), [&]() {
      return received.header.sequence == 44U;
    }));
    EXPECT_EQ(received.header.payload_size, sizeof(reboot_request));
    aerial::vehicle::RebootRequest decoded_reboot{};
    ASSERT_TRUE(aerial::vehicle::getPayload(received, MessageId::REBOOT_REQUEST,
                                            decoded_reboot));
    EXPECT_EQ(decoded_reboot.request_id, reboot_request.request_id);
  }
  for (size_t attempt = 0U; attempt < 20U && first.pendingRequestCount() != 0U;
       ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  const auto statistics = first.statistics();
  EXPECT_GT(statistics.received_frames, 0U);
  EXPECT_GT(statistics.transmitted_frames, 0U);
  EXPECT_GT(statistics.ack_received, 0U);
  EXPECT_EQ(statistics.pending_requests, 0U);

  aerial::vehicle::CommunicationMonitor monitor(first);
  monitor.noteFlightStatus(3U, 3U, 3U);
  monitor.noteRcStatus(true, 91U, -52);
  const auto communication = monitor.sample();
  EXPECT_EQ(communication.micro_usb.state,
            aerial::vehicle::CommunicationLinkState::CONNECTED);
  EXPECT_EQ(communication.ros.state,
            aerial::vehicle::CommunicationLinkState::CONNECTED);
  EXPECT_EQ(communication.wifi.state,
            aerial::vehicle::CommunicationLinkState::CONNECTED);
  EXPECT_EQ(communication.rc.quality_percent, 91U);
  EXPECT_EQ(communication.ack.state,
            aerial::vehicle::CommunicationLinkState::CONNECTED);
}

TEST(SpinalDdsEndpoint, RetriesAndTimesOutUnacknowledgedRequest) {
  SpinalDdsEndpoint host;
  SpinalDdsEndpoint spinal;
  const uint32_t domain_id = 220U + static_cast<uint32_t>(getpid() % 10);
  ASSERT_TRUE(host.start("spinal_dds_timeout_host", domain_id));
  ASSERT_TRUE(spinal.start("spinal_dds_timeout_spinal", domain_id,
                           SpinalDdsEndpoint::Role::SPINAL_TEST));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  aerial::vehicle::FlightCommand command{};
  command.command = 1U;
  Frame frame;
  ASSERT_TRUE(aerial::vehicle::setPayload(frame, MessageId::FLIGHT_COMMAND,
                                          command, 1U, 0U, Source::ROS2,
                                          Reliability::RELIABLE));
  ASSERT_TRUE(host.publish(frame));
  for (size_t attempt = 0U; attempt < 30U; ++attempt) {
    if (host.statistics().ack_timeouts > 0U)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  const auto statistics = host.statistics();
  EXPECT_EQ(statistics.ack_retries, 3U);
  EXPECT_EQ(statistics.ack_timeouts, 1U);
  EXPECT_EQ(statistics.pending_requests, 0U);
}
} // namespace
