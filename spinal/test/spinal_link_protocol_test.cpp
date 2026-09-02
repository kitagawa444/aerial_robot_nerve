#include <gtest/gtest.h>

#include "communication/spinal_link_configuration_conversion.h"
#include "communication/spinal_link_delivery.h"
#include "communication/spinal_link_protocol.h"
#include "communication/spinal_link_topics.generated.h"

TEST(SpinalLinkProtocol, RoundTripsFlightCommand) {
  aerial::vehicle::FlightCommand command;
  command.command_id = 42U;
  command.issued_at_ms = 1234U;
  command.command = 3U;
  command.source = static_cast<uint8_t>(aerial::vehicle::Source::MAVLINK);
  command.target_system = 1U;
  command.target_component = 1U;

  aerial::vehicle::Frame frame;
  ASSERT_TRUE(aerial::vehicle::setPayload(
      frame, aerial::vehicle::MessageId::FLIGHT_COMMAND, command, 5U, 9000U,
      aerial::vehicle::Source::MAVLINK,
      aerial::vehicle::Reliability::RELIABLE));

  EXPECT_TRUE(aerial::vehicle::validFrame(frame));
  EXPECT_EQ(frame.header.sequence, 5U);
  EXPECT_EQ(frame.header.timestamp_us, 9000U);

  aerial::vehicle::FlightCommand decoded;
  ASSERT_TRUE(aerial::vehicle::getPayload(
      frame, aerial::vehicle::MessageId::FLIGHT_COMMAND, decoded));
  EXPECT_EQ(decoded.command_id, command.command_id);
  EXPECT_EQ(decoded.command, command.command);
  EXPECT_EQ(decoded.source, command.source);
}

TEST(SpinalLinkProtocol, LoadsPerMessageTopicPolicy) {
  const auto *imu =
      aerial::vehicle::enabledTopicDescriptor(aerial::vehicle::MessageId::IMU);
  ASSERT_NE(imu, nullptr);
  EXPECT_EQ(imu->reliability, aerial::vehicle::Reliability::BEST_EFFORT);
  EXPECT_FALSE(imu->application_ack);

  const auto *command = aerial::vehicle::enabledTopicDescriptor(
      aerial::vehicle::MessageId::FLIGHT_COMMAND);
  ASSERT_NE(command, nullptr);
  EXPECT_EQ(command->reliability, aerial::vehicle::Reliability::RELIABLE);
  EXPECT_TRUE(command->application_ack);
  EXPECT_TRUE(command->deduplicate);
  EXPECT_GT(command->ack_timeout_ms, 0U);

  EXPECT_EQ(aerial::vehicle::enabledTopicDescriptor(
                aerial::vehicle::MessageId::MOTOR_OUTPUTS),
            nullptr);
}

TEST(SpinalLinkProtocol, DeduplicatesBySourceMessageAndRequestId) {
  aerial::vehicle::RequestDeduplicator<2U> deduplicator;
  aerial::vehicle::FlightCommand command{};
  aerial::vehicle::Frame first;
  ASSERT_TRUE(aerial::vehicle::setPayload(
      first, aerial::vehicle::MessageId::FLIGHT_COMMAND, command, 1U, 0U,
      aerial::vehicle::Source::ROS2, aerial::vehicle::Reliability::RELIABLE,
      77U));
  EXPECT_FALSE(deduplicator.seen(first));
  deduplicator.remember(first);
  EXPECT_TRUE(deduplicator.seen(first));

  aerial::vehicle::Frame other_source = first;
  other_source.header.source =
      static_cast<uint8_t>(aerial::vehicle::Source::MAVLINK);
  EXPECT_FALSE(deduplicator.seen(other_source));

  aerial::vehicle::Frame other_request = first;
  other_request.header.request_id = 78U;
  EXPECT_FALSE(deduplicator.seen(other_request));
}

TEST(SpinalLinkProtocol, RejectsWrongMessageType) {
  aerial::vehicle::Heartbeat heartbeat;
  aerial::vehicle::Frame frame;
  ASSERT_TRUE(aerial::vehicle::setPayload(
      frame, aerial::vehicle::MessageId::HEARTBEAT, heartbeat, 1U, 0U,
      aerial::vehicle::Source::INTERNAL,
      aerial::vehicle::Reliability::BEST_EFFORT));

  aerial::vehicle::FlightCommand command;
  EXPECT_FALSE(aerial::vehicle::getPayload(
      frame, aerial::vehicle::MessageId::FLIGHT_COMMAND, command));
}

TEST(SpinalLinkProtocol, ConvertsVariableCountConfigurationWithoutNativeAbi) {
  PositionControlConfig source;
  source.motor_count = 3U;
  source.use_lqi_gains = true;
  source.position_p[0] = 1.25F;
  source.vertical_acceleration_to_thrust[2] = 4.5F;
  source.supervisor.takeoff_height = 1.75F;

  const auto wire = aerial::vehicle::conversion::toWire(source);
  EXPECT_EQ(wire.motor_count, 3U);
  EXPECT_EQ(sizeof(wire), 552U);

  const PositionControlConfig decoded =
      aerial::vehicle::conversion::toInternal(wire);
  EXPECT_EQ(decoded.motor_count, 3U);
  EXPECT_TRUE(decoded.use_lqi_gains);
  EXPECT_FLOAT_EQ(decoded.position_p[0], 1.25F);
  EXPECT_FLOAT_EQ(decoded.vertical_acceleration_to_thrust[2], 4.5F);
  EXPECT_FLOAT_EQ(decoded.supervisor.takeoff_height, 1.75F);
}

TEST(SpinalLinkProtocol, ConvertsFlightParameterSnapshot) {
  FlightParameterPayload source;
  source.valid_fields =
      FlightParameterField::AIRFRAME | FlightParameterField::POSITION_CONTROL;
  source.motor_count = 4U;
  source.uav_model = 16;
  source.position_control.motor_count = 4U;
  source.position_control.yaw_p_gain[3] = 9.0F;

  const auto wire = aerial::vehicle::conversion::toWire(source);
  EXPECT_EQ(sizeof(wire), 1368U);
  const FlightParameterPayload decoded =
      aerial::vehicle::conversion::toInternal(wire);
  EXPECT_EQ(decoded.valid_fields, source.valid_fields);
  EXPECT_EQ(decoded.motor_count, 4U);
  EXPECT_EQ(decoded.uav_model, 16);
  EXPECT_EQ(decoded.position_control.motor_count, 4U);
  EXPECT_FLOAT_EQ(decoded.position_control.yaw_p_gain[3], 9.0F);
}
