#pragma once

#include <cstdint>

#include "flight_control/health/health_manager.h"

namespace FlightCommandSource
{
enum : uint8_t
{
  NONE = 0,
  ROS = 1,
  RC = 2,
  INTERNAL = 3,
  FAILSAFE = 4,
  MAVLINK = 5
};
}

namespace FlightArmingState
{
enum : uint8_t
{
  DISARMED = 0,
  ARMED = 1
};
}

namespace FlightPhase
{
enum : uint8_t
{
  LANDED = 0,
  TAKING_OFF = 1,
  AIRBORNE = 2,
  LANDING = 3
};
}

namespace FlightControlMode
{
enum : uint8_t
{
  IDLE = 0,
  ATTITUDE = 1,
  POSITION = 2,
  TAKEOFF = 3,
  LAND = 4,
  FORCE_LAND = 5,
  TERMINATED = 6
};
}

namespace FlightFailsafeState
{
enum : uint8_t
{
  NONE = 0,
  FORCE_LAND = 1,
  TERMINATED = 2
};
}

namespace FlightTransitionReason
{
enum : uint8_t
{
  NONE = 0,
  COMMAND_ACCEPTED = 1,
  TAKEOFF_COMPLETE = 2,
  LANDING_COMPLETE = 3,
  POSITION_LOST = 4,
  CONTROLLER_REPORTED_FORCE_LAND = 5,
  HEALTH_FORCE_LAND = 6,
  HEALTH_TERMINATE = 7
};
}

namespace FlightLinkState
{
enum : uint8_t
{
  UNKNOWN = HealthLinkState::UNKNOWN,
  DISCONNECTED = HealthLinkState::DISCONNECTED,
  CONNECTING = HealthLinkState::CONNECTING,
  CONNECTED = HealthLinkState::CONNECTED,
  STALE = HealthLinkState::STALE,
  DISABLED = HealthLinkState::DISABLED
};
}

namespace FlightCommandResult
{
enum : uint8_t
{
  ACCEPTED = 0,
  UNSUPPORTED = 1,
  ALREADY_IN_STATE = 2,
  NOT_ARMED = 3,
  NOT_LANDED = 4,
  ATTITUDE_NOT_READY = 5,
  POSITION_NOT_READY = 6,
  IN_AIR = 7,
  HEALTH_INHIBIT = 8
};
}

struct FlightSupervisorConfig
{
  float takeoff_height{ 1.0f };
  float takeoff_position_tolerance{ 0.15f };
  float takeoff_velocity_tolerance{ 0.2f };
  uint32_t takeoff_stable_time_ms{ 1000U };
  float landing_speed{ -0.3f };
  float landed_height{ 0.12f };
  float landed_velocity{ 0.2f };
  uint32_t landed_stable_time_ms{ 1000U };
  uint32_t rc_authority_timeout_ms{ 500U };
};

struct FlightSupervisorInput
{
  uint32_t now_ms{ 0U };
  bool attitude_ready{ false };
  bool position_control_enabled{ false };
  bool position_ready{ false };
  bool estimation_ready{ false };
  uint8_t estimation_validity{ 0U };
  uint32_t rejected_measurements{ 0U };
  bool actuator_ready{ false };
  bool config_ready{ false };
  bool power_monitor_present{ false };
  float battery_voltage{ -1.0f };
  bool primary_imu_present{ false };
  uint32_t last_primary_imu_update_ms{ 0U };
  bool control_loop_period_valid{ false };
  uint32_t control_loop_period_ms{ 0U };
  uint32_t control_loop_sequence{ 0U };
  bool rc_connected{ false };
  uint8_t ros_link_state{ FlightLinkState::UNKNOWN };
  uint8_t network_link_state{ FlightLinkState::UNKNOWN };
  uint8_t rc_link_state{ FlightLinkState::DISCONNECTED };
  bool rc_active{ false };
  float vertical_position{ 0.0f };
  float vertical_velocity{ 0.0f };
};

struct FlightSupervisorStatus
{
  uint32_t sequence{ 0U };
  uint32_t command_sequence{ 0U };
  uint8_t arming_state{ FlightArmingState::DISARMED };
  uint8_t flight_phase{ FlightPhase::LANDED };
  uint8_t control_mode{ FlightControlMode::IDLE };
  uint8_t user_intention{ FlightControlMode::IDLE };
  uint8_t authority{ FlightCommandSource::NONE };
  uint8_t failsafe{ FlightFailsafeState::NONE };
  uint8_t last_command{ 0U };
  uint8_t last_command_source{ FlightCommandSource::NONE };
  uint8_t last_command_result{ FlightCommandResult::ACCEPTED };
  uint8_t transition_reason{ FlightTransitionReason::NONE };
  bool ready_to_arm{ false };
  bool attitude_ready{ false };
  bool position_ready{ false };
  bool rc_connected{ false };
  uint8_t ros_link_state{ FlightLinkState::UNKNOWN };
  uint8_t network_link_state{ FlightLinkState::UNKNOWN };
  uint8_t rc_link_state{ FlightLinkState::DISCONNECTED };
  FlightHealthReport health{};
  float takeoff_reference_height{ 0.0f };
  float takeoff_target_height{ 0.0f };
};

// FlightSupervisor is the sole writer of arming, flight phase, active mode,
// authority and failsafe state.  Sensor and link monitors provide facts through
// FlightSupervisorInput; command sources can only request transitions.
class FlightSupervisor
{
public:
  FlightSupervisor() = default;

  void reset();
  bool configure(const FlightSupervisorConfig &config);
  bool configureHealth(const HealthManagerConfig &config);
  bool request(uint8_t command, uint8_t source, const FlightSupervisorInput &input);
  void update(const FlightSupervisorInput &input);
  void reportControllerForceLand(const FlightSupervisorInput &input);
  void noteExternalSetpoint(uint8_t source = FlightCommandSource::ROS);

  const FlightSupervisorConfig &config() const { return config_; }
  const FlightSupervisorStatus &status() const { return status_; }

private:
  FlightSupervisorConfig config_{};
  FlightSupervisorStatus status_{};
  HealthManager health_manager_{};
  uint32_t stable_since_ms_{ 0U };
  uint32_t last_rc_active_ms_{ 0U };

  void updateHealth_(const FlightSupervisorInput &input);
  bool applyHealthAction_();
  void setCommandResult_(uint8_t command, uint8_t source, uint8_t result);
  void transition_(uint8_t arming_state, uint8_t flight_phase, uint8_t control_mode, uint8_t authority,
                   uint8_t failsafe, uint8_t reason);
  void enterForceLand_(uint8_t reason);
  void disarm_(uint8_t mode, uint8_t failsafe, uint8_t reason);
  static bool finite_(float value);
};
