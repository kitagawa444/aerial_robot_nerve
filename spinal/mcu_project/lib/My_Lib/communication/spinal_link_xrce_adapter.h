#pragma once

#include <cstdint>

#include "communication/spinal_link_delivery.h"
#include "communication/spinal_link_protocol.h"
#include "communication/spinal_link_topics.generated.h"
#include "xrce_dds/xrce_dds_client.h"

class BatteryStatus;
class CrsfInput;
class ConfigFlashDatabase;
struct ConfigFlashPayload;
class FlightControl;
class IMU;
class StateEstimate;
class ThrusterManager;

class SpinalLinkXrceAdapter {
public:
  void init(XrceDdsClient *client, StateEstimate *estimator,
            FlightControl *flight_control, BatteryStatus *battery,
            CrsfInput *crsf_input, ThrusterManager *thruster,
            ConfigFlashDatabase *config_flash_database,
            osMutexId *control_mutex);
  void update();

private:
  static constexpr uint32_t kReconnectIntervalMs = 1000U;
  static constexpr uint32_t kHeartbeatIntervalMs = 1000U;
  static constexpr uint32_t kImuIntervalMs = 5U;
  static constexpr uint32_t kStateIntervalMs = 20U;
  static constexpr uint32_t kFlightStatusIntervalMs = 50U;
  static constexpr uint32_t kBatteryIntervalMs = 100U;
  static constexpr uint32_t kRcIntervalMs = 100U;
  static constexpr uint32_t kMetadataIntervalMs = 1000U;
  static constexpr uint32_t kParameterTableIntervalMs = 5000U;
  static constexpr uint32_t kRosHeartbeatStaleMs = 1000U;
  static constexpr uint32_t kRosHeartbeatDisconnectedMs = 5000U;

  XrceDdsClient *client_{nullptr};
  StateEstimate *estimator_{nullptr};
  FlightControl *flight_control_{nullptr};
  BatteryStatus *battery_{nullptr};
  CrsfInput *crsf_input_{nullptr};
  ThrusterManager *thruster_{nullptr};
  ConfigFlashDatabase *config_flash_database_{nullptr};
  osMutexId *control_mutex_{nullptr};
  uint32_t sequence_{0U};
  uint32_t last_connect_attempt_ms_{0U};
  uint32_t last_heartbeat_ms_{0U};
  uint32_t last_imu_ms_{0U};
  uint32_t last_state_ms_{0U};
  uint32_t last_flight_status_ms_{0U};
  uint32_t last_battery_ms_{0U};
  uint32_t last_rc_ms_{0U};
  uint32_t last_metadata_ms_{0U};
  uint32_t last_parameter_table_ms_{0U};
  uint32_t last_flight_status_sequence_{0U};
  uint32_t last_ros_heartbeat_ms_{0U};
  bool ros_heartbeat_seen_{false};
  uint32_t configuration_update_id_{0U};
  uint16_t configuration_kind_{0U};
  uint16_t configuration_size_{0U};
  uint16_t configuration_received_{0U};
  uint8_t configuration_buffer_[600]{};
  bool reboot_requested_{false};
  bool bootloader_requested_{false};
  uint32_t reboot_request_ms_{0U};
  static constexpr size_t kProcessedRequestCount = 32U;
  aerial::vehicle::RequestDeduplicator<kProcessedRequestCount>
      request_deduplicator_{};

  static void frameCallback(const aerial::vehicle::Frame &frame,
                            void *argument);
  void handleFrame(const aerial::vehicle::Frame &frame);
  void publishProtocolAck(const aerial::vehicle::Frame &frame, bool accepted,
                          bool duplicate);
  void publishPeriodic(uint32_t now_ms);
  void publishHeartbeat(uint32_t now_ms);
  void publishImu(uint32_t now_ms);
  void publishState(uint32_t now_ms);
  void publishFlightStatus(uint32_t now_ms);
  void publishBattery(uint32_t now_ms);
  void publishRc(uint32_t now_ms);
  void publishMetadata(uint32_t now_ms);
  void publishApplicationCapabilities(uint32_t now_ms);
  void publishConfigFlashStatus(uint32_t now_ms, uint32_t request_id,
                                uint8_t result, bool success);
  void publishFlightParameterStatus(uint32_t now_ms, uint32_t request_id,
                                    uint8_t result, bool success,
                                    bool publish_table);
  void publishFlightParameterTable(uint32_t now_ms, uint32_t request_id);
  void publishConfigAck(uint32_t now_ms);
  void updateRosLinkState(uint32_t now_ms);
  void applyExternalState(
      const aerial::vehicle::ExternalStateMeasurement &measurement);
  void applyPositionSetpoint(
      const aerial::vehicle::PositionControlSetpoint &setpoint, uint8_t source);
  void applyHealthConfig(const aerial::vehicle::HealthConfig &config);
  void
  handleConfigurationChunk(const aerial::vehicle::ConfigurationChunk &chunk);
  void applyConfiguration(uint16_t kind, const uint8_t *data, size_t size);
  void
  handleConfigFlashRequest(const aerial::vehicle::ConfigFlashRequest &request);
  void handleParameterRequest(const aerial::vehicle::ParameterRequest &request);
  void handleRebootRequest(const aerial::vehicle::RebootRequest &request);
  void handleBootloaderRequest(const aerial::vehicle::RebootRequest &request);
  static aerial::vehicle::ConfigFlashSelection
  toSelection(const ConfigFlashPayload &configuration);
  void lockControl();
  void unlockControl();

  template <typename Payload>
  void publish(aerial::vehicle::MessageId message_id, const Payload &payload,
               uint32_t now_ms, aerial::vehicle::Reliability reliability) {
    if (client_ == nullptr || !client_->connected())
      return;
    aerial::vehicle::Frame frame;
    aerial::vehicle::setPayload(frame, message_id, payload, ++sequence_,
                                static_cast<uint64_t>(now_ms) * 1000ULL,
                                aerial::vehicle::Source::INTERNAL, reliability);
    (void)client_->publish(frame);
  }
};
