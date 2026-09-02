// Generated from spinal_link_topics.yaml. Do not edit manually.
#pragma once

#include <cstddef>
#include <cstdint>

#include "communication/spinal_link_protocol.h"

namespace aerial {
namespace vehicle {

enum class TopicDirection : uint8_t { SPINAL_TO_HOST = 0, HOST_TO_SPINAL = 1 };
enum class TopicDurability : uint8_t { VOLATILE = 0, TRANSIENT_LOCAL = 1 };

struct TopicDescriptor {
  MessageId message_id;
  const char *topic_name;
  TopicDirection direction;
  Reliability reliability;
  TopicDurability durability;
  uint8_t history_depth;
  bool enabled;
  bool application_ack;
  bool deduplicate;
  uint16_t ack_timeout_ms;
  uint8_t max_retries;
  uint16_t entity_id;
};

// clang-format off
constexpr TopicDescriptor kSpinalLinkTopics[] = {
  {MessageId::HEARTBEAT, "aerial/spinal_link/heartbeat", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 1U},
  {MessageId::IMU, "aerial/spinal_link/imu", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 2U},
  {MessageId::STATE_ESTIMATE, "aerial/spinal_link/state_estimate", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 3U},
  {MessageId::FLIGHT_STATUS, "aerial/spinal_link/flight_status", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, false, false, 0U, 0U, 4U},
  {MessageId::BATTERY_STATUS, "aerial/spinal_link/battery_status", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 5U},
  {MessageId::RC_STATUS, "aerial/spinal_link/rc_status", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 6U},
  {MessageId::RC_CHANNELS, "aerial/spinal_link/rc_channels", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 7U},
  {MessageId::MOTOR_OUTPUTS, "aerial/spinal_link/motor_outputs", TopicDirection::SPINAL_TO_HOST, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, false, false, false, 0U, 0U, 8U},
  {MessageId::EVENT, "aerial/spinal_link/event", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 10U, false, false, false, 0U, 0U, 9U},
  {MessageId::APPLICATION_CAPABILITIES, "aerial/spinal_link/application_capabilities", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::TRANSIENT_LOCAL, 1U, true, false, false, 0U, 0U, 10U},
  {MessageId::CONFIG_FLASH_STATUS, "aerial/spinal_link/config_flash_status", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, false, false, 0U, 0U, 11U},
  {MessageId::FLIGHT_PARAMETER_STATUS, "aerial/spinal_link/flight_parameter_status", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, false, false, 0U, 0U, 12U},
  {MessageId::FLIGHT_PARAMETER_CHUNK, "aerial/spinal_link/flight_parameter_chunk", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 10U, true, false, false, 0U, 0U, 13U},
  {MessageId::REBOOT_STATUS, "aerial/spinal_link/reboot_status", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, false, false, 0U, 0U, 14U},
  {MessageId::CONFIG_ACK, "aerial/spinal_link/config_ack", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, false, false, 0U, 0U, 15U},
  {MessageId::BOOTLOADER_STATUS, "aerial/spinal_link/bootloader_status", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, false, false, 0U, 0U, 16U},
  {MessageId::PROTOCOL_ACK, "aerial/spinal_link/protocol_ack", TopicDirection::SPINAL_TO_HOST, Reliability::RELIABLE, TopicDurability::VOLATILE, 10U, true, false, false, 0U, 0U, 17U},
  {MessageId::NETWORK_HEARTBEAT, "aerial/spinal_link/network_heartbeat", TopicDirection::HOST_TO_SPINAL, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 18U},
  {MessageId::FLIGHT_COMMAND, "aerial/spinal_link/flight_command", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, true, true, 100U, 3U, 19U},
  {MessageId::EXTERNAL_STATE_MEASUREMENT, "aerial/spinal_link/external_state_measurement", TopicDirection::HOST_TO_SPINAL, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 20U},
  {MessageId::POSITION_CONTROL_SETPOINT, "aerial/spinal_link/position_control_setpoint", TopicDirection::HOST_TO_SPINAL, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 21U},
  {MessageId::HEALTH_CONFIG, "aerial/spinal_link/health_config", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, true, true, 200U, 3U, 22U},
  {MessageId::PARAMETER_REQUEST, "aerial/spinal_link/parameter_request", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, true, true, 500U, 3U, 23U},
  {MessageId::CONFIG_FLASH_REQUEST, "aerial/spinal_link/config_flash_request", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, true, true, 500U, 3U, 24U},
  {MessageId::REBOOT_REQUEST, "aerial/spinal_link/reboot_request", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, true, true, 200U, 3U, 25U},
  {MessageId::ROS_HEARTBEAT, "aerial/spinal_link/ros_heartbeat", TopicDirection::HOST_TO_SPINAL, Reliability::BEST_EFFORT, TopicDurability::VOLATILE, 1U, true, false, false, 0U, 0U, 26U},
  {MessageId::CONFIGURATION_CHUNK, "aerial/spinal_link/configuration_chunk", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 10U, true, true, true, 200U, 5U, 27U},
  {MessageId::BOOTLOADER_REQUEST, "aerial/spinal_link/bootloader_request", TopicDirection::HOST_TO_SPINAL, Reliability::RELIABLE, TopicDurability::VOLATILE, 5U, true, true, true, 200U, 3U, 28U},
};
constexpr size_t kSpinalLinkTopicCount =
    sizeof(kSpinalLinkTopics) / sizeof(kSpinalLinkTopics[0]);

inline const TopicDescriptor *topicDescriptor(MessageId message_id) {
  for (size_t index = 0; index < kSpinalLinkTopicCount; ++index) {
    if (kSpinalLinkTopics[index].message_id == message_id)
      return &kSpinalLinkTopics[index];
  }
  return nullptr;
}

inline const TopicDescriptor *enabledTopicDescriptor(MessageId message_id) {
  const TopicDescriptor *descriptor = topicDescriptor(message_id);
  return descriptor != nullptr && descriptor->enabled ? descriptor : nullptr;
}

static_assert(static_cast<uint16_t>(MessageId::HEARTBEAT) == 1U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::IMU) == 2U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::STATE_ESTIMATE) == 3U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::FLIGHT_STATUS) == 4U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::BATTERY_STATUS) == 5U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::RC_STATUS) == 6U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::RC_CHANNELS) == 7U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::MOTOR_OUTPUTS) == 8U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::EVENT) == 9U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::APPLICATION_CAPABILITIES) == 10U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::CONFIG_FLASH_STATUS) == 11U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::FLIGHT_PARAMETER_STATUS) == 12U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::FLIGHT_PARAMETER_CHUNK) == 13U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::REBOOT_STATUS) == 14U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::CONFIG_ACK) == 15U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::BOOTLOADER_STATUS) == 16U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::PROTOCOL_ACK) == 17U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::NETWORK_HEARTBEAT) == 100U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::FLIGHT_COMMAND) == 101U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::EXTERNAL_STATE_MEASUREMENT) == 102U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::POSITION_CONTROL_SETPOINT) == 103U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::HEALTH_CONFIG) == 104U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::PARAMETER_REQUEST) == 105U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::CONFIG_FLASH_REQUEST) == 106U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::REBOOT_REQUEST) == 107U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::ROS_HEARTBEAT) == 108U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::CONFIGURATION_CHUNK) == 109U, "Message ID mismatch");
static_assert(static_cast<uint16_t>(MessageId::BOOTLOADER_REQUEST) == 110U, "Message ID mismatch");
// clang-format on

} // namespace vehicle
} // namespace aerial
