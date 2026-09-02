#include "communication/spinal_link_xrce_adapter.h"

#include <cstring>

#include "battery_status/battery_status.h"
#include "bootloader/system_bootloader.h"
#include "communication/spinal_link_configuration_conversion.h"
#include "flashmemory/application_capabilities.h"
#include "flashmemory/config_flash_storage.h"
#include "flashmemory/flashmemory.h"
#include "flight_control/flight_control.h"
#include "flight_control/health/health_manager.h"
#include "rc/crsf_input.h"
#include "state_estimate/state_estimate.h"
#include "thruster/board/thruster_manager.h"

namespace {
constexpr uint8_t kResultOk = 0U;
constexpr uint8_t kResultInvalidCommand = 1U;
constexpr uint8_t kResultArmed = 2U;
constexpr uint8_t kResultUnsupported = 3U;
constexpr uint8_t kResultStorageError = 4U;
constexpr uint8_t kResultNoValidData = 5U;

static_assert(sizeof(aerial::vehicle::FlightParameterSnapshot) < 0x10000U,
              "Flight-parameter snapshot exceeds the chunk protocol");

void copyHealthStatus(aerial::vehicle::HealthStatus &destination,
                      const HealthComponentStatus &source) {
  destination.state = source.state;
  destination.reason_mask = source.reason_mask;
  destination.last_change_ms = source.last_change_ms;
}

template <typename T>
bool copyConfiguration(T &destination, const uint8_t *source, size_t size) {
  if (source == nullptr || size != sizeof(T))
    return false;
  std::memcpy(&destination, source, sizeof(T));
  return true;
}
} // namespace

void SpinalLinkXrceAdapter::init(XrceDdsClient *client,
                                 StateEstimate *estimator,
                                 FlightControl *flight_control,
                                 BatteryStatus *battery, CrsfInput *crsf_input,
                                 ThrusterManager *thruster,
                                 ConfigFlashDatabase *config_flash_database,
                                 osMutexId *control_mutex) {
  client_ = client;
  estimator_ = estimator;
  flight_control_ = flight_control;
  battery_ = battery;
  crsf_input_ = crsf_input;
  thruster_ = thruster;
  config_flash_database_ = config_flash_database;
  control_mutex_ = control_mutex;
  if (client_ != nullptr)
    client_->setFrameCallback(&SpinalLinkXrceAdapter::frameCallback, this);
  if (flight_control_ != nullptr)
    flight_control_->setRosLinkState(FlightLinkState::DISABLED);
}

void SpinalLinkXrceAdapter::update() {
  if (client_ == nullptr)
    return;
  const uint32_t now_ms = HAL_GetTick();
  if (!client_->connected()) {
    if (now_ms - last_connect_attempt_ms_ < kReconnectIntervalMs)
      return;
    last_connect_attempt_ms_ = now_ms;
    (void)client_->connect();
    return;
  }

  if (!client_->spin(1U)) {
    client_->disconnect();
    return;
  }
  updateRosLinkState(now_ms);
  publishPeriodic(now_ms);
  if (reboot_requested_ && now_ms - reboot_request_ms_ >= 200U) {
    if (flight_control_ != nullptr)
      flight_control_->setEnabled(false);
    if (bootloader_requested_)
      SystemBootloader::request_and_reset();
    SystemBootloader::request_application_reset();
  }
}

void SpinalLinkXrceAdapter::frameCallback(const aerial::vehicle::Frame &frame,
                                          void *argument) {
  auto *self = static_cast<SpinalLinkXrceAdapter *>(argument);
  if (self != nullptr)
    self->handleFrame(frame);
}

void SpinalLinkXrceAdapter::handleFrame(const aerial::vehicle::Frame &frame) {
  using namespace aerial::vehicle;
  const auto message_id = static_cast<MessageId>(frame.header.message_id);
  const TopicDescriptor *descriptor = enabledTopicDescriptor(message_id);
  if (descriptor == nullptr ||
      descriptor->direction != TopicDirection::HOST_TO_SPINAL)
    return;
  if (descriptor->application_ack && frame.header.request_id == 0U) {
    publishProtocolAck(frame, false, false);
    return;
  }
  if (descriptor->deduplicate && request_deduplicator_.seen(frame)) {
    publishProtocolAck(frame, true, true);
    return;
  }

  bool accepted = false;
  lockControl();
  switch (message_id) {
  case MessageId::FLIGHT_COMMAND: {
    FlightCommand command{};
    if (getPayload(frame, MessageId::FLIGHT_COMMAND, command) &&
        flight_control_ != nullptr)
      accepted = flight_control_->requestFlightCommand(command.command,
                                                       command.source);
    break;
  }
  case MessageId::EXTERNAL_STATE_MEASUREMENT: {
    ExternalStateMeasurement measurement{};
    if (getPayload(frame, MessageId::EXTERNAL_STATE_MEASUREMENT, measurement)) {
      applyExternalState(measurement);
      accepted = true;
    }
    break;
  }
  case MessageId::POSITION_CONTROL_SETPOINT: {
    aerial::vehicle::PositionControlSetpoint setpoint{};
    if (getPayload(frame, MessageId::POSITION_CONTROL_SETPOINT, setpoint)) {
      const uint8_t source =
          frame.header.source == static_cast<uint8_t>(Source::MAVLINK)
              ? FlightCommandSource::MAVLINK
              : FlightCommandSource::ROS;
      applyPositionSetpoint(setpoint, source);
      accepted = true;
    }
    break;
  }
  case MessageId::HEALTH_CONFIG: {
    HealthConfig config{};
    if (getPayload(frame, MessageId::HEALTH_CONFIG, config)) {
      applyHealthConfig(config);
      accepted = true;
    }
    break;
  }
  case MessageId::ROS_HEARTBEAT:
    if (flight_control_ != nullptr) {
      last_ros_heartbeat_ms_ = HAL_GetTick();
      ros_heartbeat_seen_ = true;
      flight_control_->setRosLinkState(FlightLinkState::CONNECTED);
      accepted = true;
    }
    break;
  case MessageId::NETWORK_HEARTBEAT:
    if (flight_control_ != nullptr) {
      flight_control_->noteNetworkHeartbeat();
      accepted = true;
    }
    break;
  case MessageId::CONFIGURATION_CHUNK: {
    ConfigurationChunk chunk{};
    if (getPayload(frame, MessageId::CONFIGURATION_CHUNK, chunk)) {
      handleConfigurationChunk(chunk);
      accepted = true;
    }
    break;
  }
  case MessageId::CONFIG_FLASH_REQUEST: {
    ConfigFlashRequest request{};
    if (getPayload(frame, MessageId::CONFIG_FLASH_REQUEST, request)) {
      handleConfigFlashRequest(request);
      accepted = true;
    }
    break;
  }
  case MessageId::PARAMETER_REQUEST: {
    ParameterRequest request{};
    if (getPayload(frame, MessageId::PARAMETER_REQUEST, request)) {
      handleParameterRequest(request);
      accepted = true;
    }
    break;
  }
  case MessageId::REBOOT_REQUEST: {
    RebootRequest request{};
    if (getPayload(frame, MessageId::REBOOT_REQUEST, request)) {
      handleRebootRequest(request);
      accepted = true;
    }
    break;
  }
  case MessageId::BOOTLOADER_REQUEST: {
    RebootRequest request{};
    if (getPayload(frame, MessageId::BOOTLOADER_REQUEST, request)) {
      handleBootloaderRequest(request);
      accepted = true;
    }
    break;
  }
  default:
    break;
  }
  unlockControl();
  if (descriptor->application_ack) {
    if (descriptor->deduplicate)
      request_deduplicator_.remember(frame);
    publishProtocolAck(frame, accepted, false);
  }
}

void SpinalLinkXrceAdapter::publishProtocolAck(
    const aerial::vehicle::Frame &frame, bool accepted, bool duplicate) {
  aerial::vehicle::ProtocolAck ack{};
  ack.request_id = frame.header.request_id;
  ack.message_id = frame.header.message_id;
  ack.request_source = frame.header.source;
  ack.result = static_cast<uint8_t>(
      frame.header.request_id == 0U
          ? aerial::vehicle::ProtocolAckResult::INVALID_REQUEST
      : accepted ? aerial::vehicle::ProtocolAckResult::ACCEPTED
                 : aerial::vehicle::ProtocolAckResult::REJECTED);
  ack.duplicate = duplicate ? 1U : 0U;
  publish(aerial::vehicle::MessageId::PROTOCOL_ACK, ack, HAL_GetTick(),
          aerial::vehicle::Reliability::RELIABLE);
}

void SpinalLinkXrceAdapter::publishPeriodic(uint32_t now_ms) {
  if (now_ms - last_heartbeat_ms_ >= kHeartbeatIntervalMs)
    publishHeartbeat(now_ms);
  if (now_ms - last_imu_ms_ >= kImuIntervalMs)
    publishImu(now_ms);
  if (now_ms - last_state_ms_ >= kStateIntervalMs)
    publishState(now_ms);
  if (now_ms - last_flight_status_ms_ >= kFlightStatusIntervalMs)
    publishFlightStatus(now_ms);
  if (now_ms - last_battery_ms_ >= kBatteryIntervalMs)
    publishBattery(now_ms);
  if (now_ms - last_rc_ms_ >= kRcIntervalMs)
    publishRc(now_ms);
  if (now_ms - last_metadata_ms_ >= kMetadataIntervalMs)
    publishMetadata(now_ms);
  publishConfigAck(now_ms);
}

void SpinalLinkXrceAdapter::publishHeartbeat(uint32_t now_ms) {
  aerial::vehicle::Heartbeat heartbeat{};
  heartbeat.uptime_ms = now_ms;
  heartbeat.link_state = client_ != nullptr && client_->connected()
                             ? FlightLinkState::CONNECTED
                             : FlightLinkState::DISCONNECTED;
  publish(aerial::vehicle::MessageId::HEARTBEAT, heartbeat, now_ms,
          aerial::vehicle::Reliability::BEST_EFFORT);
  last_heartbeat_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishImu(uint32_t now_ms) {
  if (estimator_ == nullptr || estimator_->getAttEstimator() == nullptr)
    return;
  lockControl();
  AttitudeEstimate *attitude = estimator_->getAttEstimator();
  const ap::Vector3f acceleration = attitude->getAccVec();
  const ap::Vector3f angular_velocity = attitude->getGyroVec();
  const ap::Vector3f magnetic_field = attitude->getMagVec();
  const ap::Quaternion quaternion = attitude->getQuaternion();
  aerial::vehicle::Imu payload{};
  payload.acceleration[0] = acceleration.x;
  payload.acceleration[1] = acceleration.y;
  payload.acceleration[2] = acceleration.z;
  payload.angular_velocity[0] = angular_velocity.x;
  payload.angular_velocity[1] = angular_velocity.y;
  payload.angular_velocity[2] = angular_velocity.z;
  payload.magnetic_field[0] = magnetic_field.x;
  payload.magnetic_field[1] = magnetic_field.y;
  payload.magnetic_field[2] = magnetic_field.z;
  payload.attitude_xyzw[0] = quaternion.q2;
  payload.attitude_xyzw[1] = quaternion.q3;
  payload.attitude_xyzw[2] = quaternion.q4;
  payload.attitude_xyzw[3] = quaternion.q1;
  unlockControl();
  publish(aerial::vehicle::MessageId::IMU, payload, now_ms,
          aerial::vehicle::Reliability::BEST_EFFORT);
  last_imu_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishState(uint32_t now_ms) {
  if (estimator_ == nullptr || !estimator_->outputInitialized())
    return;
  lockControl();
  const Eskf15::State &state = estimator_->outputState();
  const ap::Vector3f acceleration = estimator_->getLinearAccelerationWorld();
  const ap::Vector3f angular_velocity = estimator_->getAngularRate();
  aerial::vehicle::StateEstimate payload{};
  payload.validity = estimator_->stateValidity();
  for (size_t axis = 0; axis < 3U; ++axis) {
    payload.position[axis] = state.position[axis];
    payload.velocity[axis] = state.velocity[axis];
    payload.acceleration[axis] = acceleration[axis];
    payload.angular_velocity[axis] = angular_velocity[axis];
    payload.accelerometer_bias[axis] = state.accelerometer_bias[axis];
    payload.gyroscope_bias[axis] = state.gyroscope_bias[axis];
  }
  payload.attitude_xyzw[0] = state.attitude.q2;
  payload.attitude_xyzw[1] = state.attitude.q3;
  payload.attitude_xyzw[2] = state.attitude.q4;
  payload.attitude_xyzw[3] = state.attitude.q1;
  for (size_t index = 0; index < Eskf15::STATE_SIZE; ++index)
    payload.covariance_diagonal[index] = estimator_->outputCovariance(index);
  payload.rejected_measurements = estimator_->outputRejectedMeasurementCount();
  unlockControl();
  publish(aerial::vehicle::MessageId::STATE_ESTIMATE, payload, now_ms,
          aerial::vehicle::Reliability::BEST_EFFORT);
  last_state_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishFlightStatus(uint32_t now_ms) {
  if (flight_control_ == nullptr)
    return;
  lockControl();
  const FlightSupervisorStatus &status =
      flight_control_->getSupervisor().status();
  if (status.sequence == last_flight_status_sequence_ &&
      now_ms - last_flight_status_ms_ < kHeartbeatIntervalMs) {
    unlockControl();
    return;
  }
  aerial::vehicle::FlightStatus payload{};
  payload.sequence = status.sequence;
  payload.command_sequence = status.command_sequence;
  payload.arming_state = status.arming_state;
  payload.flight_phase = status.flight_phase;
  payload.control_mode = status.control_mode;
  payload.user_intention = status.user_intention;
  payload.authority = status.authority;
  payload.failsafe = status.failsafe;
  payload.last_command = status.last_command;
  payload.last_command_source = status.last_command_source;
  payload.last_command_result = status.last_command_result;
  payload.transition_reason = status.transition_reason;
  payload.ready_to_arm = status.ready_to_arm;
  payload.attitude_ready = status.attitude_ready;
  payload.position_ready = status.position_ready;
  payload.rc_connected = status.rc_connected;
  payload.ros_link_state = status.ros_link_state;
  payload.network_link_state = status.network_link_state;
  payload.rc_link_state = status.rc_link_state;
  copyHealthStatus(payload.health.overall, status.health.overall);
  copyHealthStatus(payload.health.flight, status.health.flight);
  copyHealthStatus(payload.health.link, status.health.link.summary);
  copyHealthStatus(payload.health.power, status.health.power);
  copyHealthStatus(payload.health.sensor, status.health.sensor);
  copyHealthStatus(payload.health.estimation, status.health.estimation);
  copyHealthStatus(payload.health.actuator, status.health.actuator);
  copyHealthStatus(payload.health.compute, status.health.compute);
  copyHealthStatus(payload.health.config, status.health.config);
  payload.health.ros_link_state = status.health.link.ros;
  payload.health.network_link_state = status.health.link.network;
  payload.health.rc_link_state = status.health.link.rc;
  payload.health.requested_action = status.health.requested_action;
  payload.health.action_source = status.health.action_source;
  payload.health.action_reason_mask = status.health.action_reason_mask;
  payload.health.action_since_ms = status.health.action_since_ms;
  payload.health.battery_voltage = status.health.battery_voltage;
  payload.health.battery_compensated_voltage =
      status.health.battery_compensated_voltage;
  payload.health.battery_percentage = status.health.battery_percentage;
  payload.health.estimation_validity = status.health.estimation_validity;
  payload.health.rejected_measurements = status.health.rejected_measurements;
  payload.takeoff_reference_height = status.takeoff_reference_height;
  payload.takeoff_target_height = status.takeoff_target_height;
  last_flight_status_sequence_ = status.sequence;
  unlockControl();
  publish(aerial::vehicle::MessageId::FLIGHT_STATUS, payload, now_ms,
          aerial::vehicle::Reliability::RELIABLE);
  last_flight_status_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishBattery(uint32_t now_ms) {
  if (battery_ == nullptr)
    return;
  aerial::vehicle::BatteryStatus payload{};
  payload.voltage = battery_->getVoltage();
  if (flight_control_ != nullptr) {
    lockControl();
    const FlightHealthReport &health =
        flight_control_->getSupervisor().status().health;
    payload.compensated_voltage = health.battery_compensated_voltage;
    payload.percentage = health.battery_percentage;
    unlockControl();
  }
  publish(aerial::vehicle::MessageId::BATTERY_STATUS, payload, now_ms,
          aerial::vehicle::Reliability::BEST_EFFORT);
  last_battery_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishRc(uint32_t now_ms) {
  if (crsf_input_ == nullptr)
    return;
  CrsfInput::Snapshot snapshot{};
  if (!crsf_input_->snapshot(snapshot))
    return;
  aerial::vehicle::RcStatus status{};
  status.connected = snapshot.connected;
  status.link_quality = snapshot.link.uplink_link_quality;
  status.rssi_dbm = snapshot.link.uplink_rssi_dbm;
  publish(aerial::vehicle::MessageId::RC_STATUS, status, now_ms,
          aerial::vehicle::Reliability::BEST_EFFORT);
  aerial::vehicle::RcChannels channels{};
  for (size_t index = 0; index < crsf::CHANNEL_COUNT; ++index)
    channels.channels[index] = snapshot.raw[index];
  publish(aerial::vehicle::MessageId::RC_CHANNELS, channels, now_ms,
          aerial::vehicle::Reliability::BEST_EFFORT);
  last_rc_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishMetadata(uint32_t now_ms) {
  publishApplicationCapabilities(now_ms);
  publishConfigFlashStatus(now_ms, 0U, kResultOk, true);
  publishFlightParameterStatus(now_ms, 0U, kResultOk, true,
                               now_ms - last_parameter_table_ms_ >=
                                   kParameterTableIntervalMs);
  last_metadata_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishApplicationCapabilities(uint32_t now_ms) {
  aerial::vehicle::ApplicationCapabilities payload{};
  payload.board = 1U;
  payload.capability_mask = ApplicationCapability::mask();
  payload.max_motor_count = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  payload.config_schema_version = ConfigFlashDatabase::SCHEMA_VERSION;
  payload.flight_parameter_schema_version =
      FlightParameterDatabase::SCHEMA_VERSION;
  publish(aerial::vehicle::MessageId::APPLICATION_CAPABILITIES, payload, now_ms,
          aerial::vehicle::Reliability::RELIABLE);
}

aerial::vehicle::ConfigFlashSelection
SpinalLinkXrceAdapter::toSelection(const ConfigFlashPayload &configuration) {
  aerial::vehicle::ConfigFlashSelection selection{};
  selection.uart3_driver = configuration.uart3_driver;
  selection.imu_driver = configuration.imu_driver;
  selection.barometer_enabled = configuration.barometer_enabled;
  selection.servo_driver = configuration.servo_driver;
  selection.attitude_estimation_enabled =
      configuration.attitude_estimation_enabled;
  selection.height_estimation_enabled = configuration.height_estimation_enabled;
  selection.position_estimation_enabled =
      configuration.position_estimation_enabled;
  selection.flight_control_enabled = configuration.flight_control_enabled;
  selection.motor_output_driver = configuration.motor_output_driver;
  return selection;
}

void SpinalLinkXrceAdapter::publishConfigFlashStatus(uint32_t now_ms,
                                                     uint32_t request_id,
                                                     uint8_t result,
                                                     bool success) {
  aerial::vehicle::ConfigFlashStatus payload{};
  payload.request_id = request_id;
  payload.result = result;
  payload.success = success ? 1U : 0U;
  payload.persistent_storage = 1U;
  if (config_flash_database_ != nullptr) {
    payload.valid = config_flash_database_->valid() ? 1U : 0U;
    payload.dirty = config_flash_database_->dirty() ? 1U : 0U;
    payload.reboot_required =
        config_flash_database_->rebootRequired() ? 1U : 0U;
    payload.active_slot = config_flash_database_->activeSlot();
    payload.schema_version = config_flash_database_->schemaVersion();
    payload.generation = config_flash_database_->generation();
    payload.crc32 = config_flash_database_->crc32();
    payload.active = toSelection(config_flash_database_->activeConfiguration());
    payload.pending =
        toSelection(config_flash_database_->pendingConfiguration());
  }
  publish(aerial::vehicle::MessageId::CONFIG_FLASH_STATUS, payload, now_ms,
          aerial::vehicle::Reliability::RELIABLE);
}

void SpinalLinkXrceAdapter::publishFlightParameterStatus(uint32_t now_ms,
                                                         uint32_t request_id,
                                                         uint8_t result,
                                                         bool success,
                                                         bool publish_table) {
  if (flight_control_ == nullptr)
    return;
  const FlightParameterDatabase &database =
      flight_control_->parameterDatabase();
  aerial::vehicle::FlightParameterStatus payload{};
  payload.request_id = request_id;
  payload.result = result;
  payload.success = success ? 1U : 0U;
  payload.valid = database.valid() ? 1U : 0U;
  payload.dirty = database.dirty() ? 1U : 0U;
  payload.applied = flight_control_->parametersApplied() ? 1U : 0U;
  payload.persistent_storage = 1U;
  payload.schema_version = database.schemaVersion();
  payload.generation = database.generation();
  payload.crc32 = database.crc32();
  payload.valid_fields = database.validFields();
  payload.payload_size = sizeof(aerial::vehicle::FlightParameterSnapshot);
  publish(aerial::vehicle::MessageId::FLIGHT_PARAMETER_STATUS, payload, now_ms,
          aerial::vehicle::Reliability::RELIABLE);
  if (publish_table)
    publishFlightParameterTable(now_ms, request_id);
}

void SpinalLinkXrceAdapter::publishFlightParameterTable(uint32_t now_ms,
                                                        uint32_t request_id) {
  if (flight_control_ == nullptr)
    return;
  const aerial::vehicle::FlightParameterSnapshot parameters =
      aerial::vehicle::conversion::toWire(
          flight_control_->parameterDatabase().payload());
  const auto *bytes = reinterpret_cast<const uint8_t *>(&parameters);
  for (size_t offset = 0U; offset < sizeof(parameters);
       offset += aerial::vehicle::kChunkDataSize) {
    aerial::vehicle::FlightParameterChunk chunk{};
    chunk.request_id = request_id;
    chunk.total_size = sizeof(parameters);
    chunk.offset = static_cast<uint16_t>(offset);
    const size_t remaining = sizeof(parameters) - offset;
    chunk.chunk_size =
        static_cast<uint16_t>(remaining < aerial::vehicle::kChunkDataSize
                                  ? remaining
                                  : aerial::vehicle::kChunkDataSize);
    std::memcpy(chunk.data, bytes + offset, chunk.chunk_size);
    publish(aerial::vehicle::MessageId::FLIGHT_PARAMETER_CHUNK, chunk, now_ms,
            aerial::vehicle::Reliability::RELIABLE);
  }
  last_parameter_table_ms_ = now_ms;
}

void SpinalLinkXrceAdapter::publishConfigAck(uint32_t now_ms) {
  if (flight_control_ == nullptr)
    return;
  uint8_t command = 0U;
  if (!flight_control_->consumeConfigAck(command))
    return;
  aerial::vehicle::ConfigAck payload{};
  payload.command = command;
  publish(aerial::vehicle::MessageId::CONFIG_ACK, payload, now_ms,
          aerial::vehicle::Reliability::RELIABLE);
}

void SpinalLinkXrceAdapter::handleConfigurationChunk(
    const aerial::vehicle::ConfigurationChunk &chunk) {
  if (chunk.chunk_size > aerial::vehicle::kChunkDataSize ||
      chunk.total_size > sizeof(configuration_buffer_) ||
      static_cast<size_t>(chunk.offset) + chunk.chunk_size > chunk.total_size) {
    return;
  }
  if (chunk.offset == 0U) {
    configuration_update_id_ = chunk.update_id;
    configuration_kind_ = chunk.kind;
    configuration_size_ = chunk.total_size;
    configuration_received_ = 0U;
  }
  if (chunk.update_id != configuration_update_id_ ||
      chunk.kind != configuration_kind_ ||
      chunk.total_size != configuration_size_ ||
      chunk.offset != configuration_received_) {
    return;
  }
  std::memcpy(configuration_buffer_ + chunk.offset, chunk.data,
              chunk.chunk_size);
  configuration_received_ += chunk.chunk_size;
  if (configuration_received_ == configuration_size_) {
    applyConfiguration(configuration_kind_, configuration_buffer_,
                       configuration_size_);
    configuration_received_ = 0U;
  }
}

void SpinalLinkXrceAdapter::applyConfiguration(uint16_t kind,
                                               const uint8_t *data,
                                               size_t size) {
  using aerial::vehicle::ConfigurationKind;
  if (flight_control_ == nullptr)
    return;
  switch (static_cast<ConfigurationKind>(kind)) {
  case ConfigurationKind::AIRFRAME: {
    aerial::vehicle::AirframeConfig config{};
    if (copyConfiguration(config, data, size))
      flight_control_->applyUavInfo(config.motor_count, config.uav_model);
    break;
  }
  case ConfigurationKind::GIMBAL_DOF: {
    uint8_t gimbal_dof = 0U;
    if (copyConfiguration(gimbal_dof, data, size))
      flight_control_->applyGimbalDof(gimbal_dof);
    break;
  }
  case ConfigurationKind::FOUR_AXIS_COMMAND: {
    aerial::vehicle::FourAxisConfiguration wire{};
    if (copyConfiguration(wire, data, size))
      (void)flight_control_->applyFourAxisCommand(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::PWM_INFO: {
    aerial::vehicle::PwmConfiguration wire{};
    if (thruster_ != nullptr && copyConfiguration(wire, data, size))
      (void)thruster_->applyPwmInfo(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::ATTITUDE_GAINS: {
    aerial::vehicle::RpyGainsConfiguration wire{};
    if (copyConfiguration(wire, data, size))
      (void)flight_control_->applyRpyGains(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::P_MATRIX: {
    aerial::vehicle::PMatrixConfiguration wire{};
    if (copyConfiguration(wire, data, size))
      (void)flight_control_->applyPMatrixInertia(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::TORQUE_ALLOCATION: {
    aerial::vehicle::TorqueAllocationConfiguration wire{};
    if (copyConfiguration(wire, data, size))
      (void)flight_control_->applyTorqueAllocationMatrixInv(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::OFFSET_ROTATION: {
    aerial::vehicle::OffsetRotationConfiguration wire{};
    if (copyConfiguration(wire, data, size))
      flight_control_->applyOffsetRotation(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::POSITION_CONTROL: {
    aerial::vehicle::PositionControlConfiguration wire{};
    if (copyConfiguration(wire, data, size))
      (void)flight_control_->applyPositionControlConfig(
          aerial::vehicle::conversion::toInternal(wire));
    break;
  }
  case ConfigurationKind::ATTITUDE_CONTROL_ENABLED: {
    uint8_t enabled = 0U;
    if (copyConfiguration(enabled, data, size))
      flight_control_->setAttitudeControlFlag(enabled != 0U);
    break;
  }
  case ConfigurationKind::POSITION_CONTROL_ENABLED: {
    uint8_t enabled = 0U;
    if (copyConfiguration(enabled, data, size))
      flight_control_->setPositionControlEnabled(enabled != 0U);
    break;
  }
  }
}

void SpinalLinkXrceAdapter::handleConfigFlashRequest(
    const aerial::vehicle::ConfigFlashRequest &request) {
  bool success = false;
  uint8_t result = kResultInvalidCommand;
  const bool armed = flight_control_ != nullptr &&
                     flight_control_->getSupervisor().status().arming_state ==
                         FlightArmingState::ARMED;
  if (config_flash_database_ == nullptr) {
    result = kResultStorageError;
  } else if (request.command == 0U) {
    success = true;
    result = kResultOk;
  } else if (armed) {
    result = kResultArmed;
  } else if (request.command == 1U) {
    ConfigFlashPayload pending = config_flash_database_->pendingConfiguration();
    pending.uart3_driver = request.selection.uart3_driver;
    pending.imu_driver = request.selection.imu_driver;
    pending.barometer_enabled = request.selection.barometer_enabled;
    pending.servo_driver = request.selection.servo_driver;
    pending.attitude_estimation_enabled =
        request.selection.attitude_estimation_enabled;
    pending.height_estimation_enabled =
        request.selection.height_estimation_enabled;
    pending.position_estimation_enabled =
        request.selection.position_estimation_enabled;
    pending.flight_control_enabled = request.selection.flight_control_enabled;
    pending.motor_output_driver = request.selection.motor_output_driver;
    success = config_flash_database_->stageConfiguration(
        pending, ApplicationCapability::support());
    result = success ? kResultOk : kResultUnsupported;
  } else if (request.command == 2U) {
    success = ConfigFlashStorage::commit(*config_flash_database_);
    result = success ? kResultOk : kResultStorageError;
  } else if (request.command == 3U) {
    success = ConfigFlashStorage::reload(*config_flash_database_);
    result = success ? kResultOk : kResultNoValidData;
  }
  publishConfigFlashStatus(HAL_GetTick(), request.request_id, result, success);
}

void SpinalLinkXrceAdapter::handleParameterRequest(
    const aerial::vehicle::ParameterRequest &request) {
  bool success = false;
  uint8_t result = kResultInvalidCommand;
  const bool armed = flight_control_ != nullptr &&
                     flight_control_->getSupervisor().status().arming_state ==
                         FlightArmingState::ARMED;
  if (flight_control_ == nullptr) {
    result = kResultStorageError;
  } else if (request.command == 0U) {
    success = true;
    result = kResultOk;
  } else if (armed) {
    result = kResultArmed;
  } else if (request.command == 1U) {
    if (!flight_control_->prepareParameterCommit()) {
      result = kResultUnsupported;
    } else {
      const HAL_StatusTypeDef erase_status = FlashMemory::erase();
      const HAL_StatusTypeDef write_status =
          erase_status == HAL_OK ? FlashMemory::write() : erase_status;
      const HAL_StatusTypeDef read_status =
          write_status == HAL_OK ? FlashMemory::read() : write_status;
      success =
          read_status == HAL_OK && flight_control_->reloadParameterDatabase();
      result = success ? kResultOk : kResultStorageError;
      if (!success)
        flight_control_->getParameterDatabase().markCommitFailed();
    }
  } else if (request.command == 2U) {
    success = FlashMemory::read() == HAL_OK &&
              flight_control_->reloadParameterDatabase();
    result = success ? kResultOk : kResultNoValidData;
  }
  publishFlightParameterStatus(HAL_GetTick(), request.request_id, result,
                               success, true);
}

void SpinalLinkXrceAdapter::handleRebootRequest(
    const aerial::vehicle::RebootRequest &request) {
  aerial::vehicle::RebootStatus response{};
  response.request_id = request.request_id;
  const bool armed = flight_control_ != nullptr &&
                     flight_control_->getSupervisor().status().arming_state ==
                         FlightArmingState::ARMED;
  response.success = armed ? 0U : 1U;
  response.result = armed ? kResultArmed : kResultOk;
  publish(aerial::vehicle::MessageId::REBOOT_STATUS, response, HAL_GetTick(),
          aerial::vehicle::Reliability::RELIABLE);
  if (!armed) {
    if (thruster_ != nullptr)
      thruster_->stopOutputs();
    reboot_requested_ = true;
    bootloader_requested_ = false;
    reboot_request_ms_ = HAL_GetTick();
  }
}

void SpinalLinkXrceAdapter::handleBootloaderRequest(
    const aerial::vehicle::RebootRequest &request) {
  aerial::vehicle::RebootStatus response{};
  response.request_id = request.request_id;
  const bool armed = flight_control_ != nullptr &&
                     flight_control_->getSupervisor().status().arming_state ==
                         FlightArmingState::ARMED;
  response.success = armed ? 0U : 1U;
  response.result = armed ? kResultArmed : kResultOk;
  publish(aerial::vehicle::MessageId::BOOTLOADER_STATUS, response,
          HAL_GetTick(), aerial::vehicle::Reliability::RELIABLE);
  if (!armed) {
    if (thruster_ != nullptr)
      thruster_->stopOutputs();
    reboot_requested_ = true;
    bootloader_requested_ = true;
    reboot_request_ms_ = HAL_GetTick();
  }
}

void SpinalLinkXrceAdapter::updateRosLinkState(uint32_t now_ms) {
  if (flight_control_ == nullptr || !ros_heartbeat_seen_)
    return;
  const uint32_t age_ms = now_ms - last_ros_heartbeat_ms_;
  const uint8_t state =
      age_ms >= kRosHeartbeatDisconnectedMs ? FlightLinkState::DISCONNECTED
      : age_ms >= kRosHeartbeatStaleMs      ? FlightLinkState::STALE
                                            : FlightLinkState::CONNECTED;
  lockControl();
  flight_control_->setRosLinkState(state);
  unlockControl();
}

void SpinalLinkXrceAdapter::applyExternalState(
    const aerial::vehicle::ExternalStateMeasurement &measurement) {
  if (estimator_ == nullptr)
    return;
  const ap::Vector3f position(measurement.position[0], measurement.position[1],
                              measurement.position[2]);
  const ap::Vector3f position_variance(measurement.position_variance[0],
                                       measurement.position_variance[1],
                                       measurement.position_variance[2]);
  const ap::Vector3f velocity(measurement.velocity[0], measurement.velocity[1],
                              measurement.velocity[2]);
  const ap::Vector3f velocity_variance(measurement.velocity_variance[0],
                                       measurement.velocity_variance[1],
                                       measurement.velocity_variance[2]);
  const ap::Quaternion attitude(
      measurement.attitude_xyzw[3], measurement.attitude_xyzw[0],
      measurement.attitude_xyzw[1], measurement.attitude_xyzw[2]);
  const ap::Vector3f attitude_variance(measurement.attitude_variance[0],
                                       measurement.attitude_variance[1],
                                       measurement.attitude_variance[2]);
  const ap::Vector3f angular_velocity(measurement.angular_velocity[0],
                                      measurement.angular_velocity[1],
                                      measurement.angular_velocity[2]);
  if (measurement.estimate_mode == 5U) {
    (void)estimator_->applyDirectExternalState(
        position, position_variance, velocity, velocity_variance, attitude,
        attitude_variance, angular_velocity, measurement.field_mask,
        HAL_GetTick());
    return;
  }
  if (measurement.estimate_mode != 3U && measurement.estimate_mode != 4U)
    return;
  estimator_->setDirectStateEnabled(false);
  if ((measurement.field_mask & 1U) != 0U)
    (void)estimator_->fuseExternalPosition(position, position_variance);
  if ((measurement.field_mask & 2U) != 0U)
    (void)estimator_->fuseExternalVelocity(velocity, velocity_variance);
  if ((measurement.field_mask & 4U) != 0U)
    (void)estimator_->fuseExternalAttitude(attitude, attitude_variance);
}

void SpinalLinkXrceAdapter::applyPositionSetpoint(
    const aerial::vehicle::PositionControlSetpoint &input, uint8_t source) {
  if (flight_control_ == nullptr)
    return;
  PositionControlSetpoint setpoint{};
  setpoint.position =
      ap::Vector3f(input.position[0], input.position[1], input.position[2]);
  setpoint.velocity =
      ap::Vector3f(input.velocity[0], input.velocity[1], input.velocity[2]);
  setpoint.acceleration = ap::Vector3f(
      input.acceleration[0], input.acceleration[1], input.acceleration[2]);
  setpoint.yaw = input.yaw;
  setpoint.yaw_rate = input.yaw_rate;
  setpoint.yaw_acceleration = input.yaw_acceleration;
  setpoint.initial_height = input.initial_height;
  setpoint.horizontal_control_mode = input.horizontal_control_mode;
  setpoint.active = input.active != 0U;
  setpoint.manual_control_allowed = input.manual_control_allowed != 0U;
  setpoint.landing = input.landing != 0U;
  (void)flight_control_->applyPositionControlSetpoint(setpoint, source);
}

void SpinalLinkXrceAdapter::applyHealthConfig(
    const aerial::vehicle::HealthConfig &input) {
  if (flight_control_ == nullptr)
    return;
  HealthManagerConfig config{};
  config.power.battery_cell_count = input.battery_cell_count;
  config.power.low_percentage = input.battery_low_percentage;
  config.power.hysteresis_percentage = input.battery_hysteresis_percentage;
  config.power.high_cell_threshold = input.battery_high_cell_threshold;
  config.power.resistance = input.battery_resistance;
  config.power.resistance_voltage_rate = input.battery_resistance_voltage_rate;
  config.power.hovering_current = input.battery_hovering_current;
  config.power.debounce_ms = input.battery_debounce_ms;
  config.sensor.primary_imu_timeout_ms = input.primary_imu_timeout_ms;
  config.compute.control_loop_deadline_ms = input.control_loop_deadline_ms;
  config.compute.miss_limit = input.control_loop_miss_limit;
  (void)flight_control_->applyHealthConfig(config);
}

void SpinalLinkXrceAdapter::lockControl() {
  if (control_mutex_ != nullptr && *control_mutex_ != nullptr)
    osMutexWait(*control_mutex_, osWaitForever);
}

void SpinalLinkXrceAdapter::unlockControl() {
  if (control_mutex_ != nullptr && *control_mutex_ != nullptr)
    osMutexRelease(*control_mutex_);
}
