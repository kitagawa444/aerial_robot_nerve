#pragma once

#include <cstdint>

namespace HealthState
{
enum : uint8_t
{
  UNKNOWN = 0,
  OK = 1,
  DEGRADED = 2,
  CRITICAL = 3,
  DISABLED = 4
};
}

namespace HealthReason
{
enum : uint32_t
{
  NONE = 0U,
  NOT_READY = 1U << 0,
  STALE = 1U << 1,
  DISCONNECTED = 1U << 2,
  INVALID = 1U << 3,
  UNCONFIGURED = 1U << 4,
  DATA_UNAVAILABLE = 1U << 5,
  FAILSAFE_ACTIVE = 1U << 6,
  TERMINATED = 1U << 7,
  LOW_VOLTAGE = 1U << 8,
  SENSOR_FAILURE = 1U << 9,
  ACTUATOR_FAILURE = 1U << 10,
  DEADLINE_MISSED = 1U << 11,
  THRESHOLD_UNCONFIGURED = 1U << 12,
  BATTERY_CELL_MISMATCH = 1U << 13
};
}

namespace HealthLinkState
{
enum : uint8_t
{
  UNKNOWN = 0,
  DISCONNECTED = 1,
  CONNECTING = 2,
  CONNECTED = 3,
  STALE = 4,
  DISABLED = 5
};
}

enum class HealthComponent : uint8_t
{
  FLIGHT = 0,
  LINK = 1,
  POWER = 2,
  SENSOR = 3,
  ESTIMATION = 4,
  ACTUATOR = 5,
  COMPUTE = 6,
  CONFIG = 7
};

namespace HealthAction
{
enum : uint8_t
{
  NONE = 0,
  INHIBIT_ARM = 1,
  FORCE_LAND = 2,
  TERMINATE = 3
};
}

struct PowerHealthConfig
{
  uint8_t battery_cell_count{ 0U };
  float low_percentage{ 0.0f };
  float hysteresis_percentage{ 2.0f };
  float high_cell_threshold{ 1.0f };
  float resistance{ 0.0f };
  float resistance_voltage_rate{ 0.0f };
  float hovering_current{ 0.0f };
  uint32_t debounce_ms{ 1000U };
};

struct SensorHealthConfig
{
  uint32_t primary_imu_timeout_ms{ 100U };
};

struct ComputeHealthConfig
{
  uint32_t control_loop_deadline_ms{ 20U };
  uint8_t miss_limit{ 3U };
};

struct HealthManagerConfig
{
  PowerHealthConfig power{};
  SensorHealthConfig sensor{};
  ComputeHealthConfig compute{};
};

struct HealthEvaluation
{
  uint8_t state{ HealthState::UNKNOWN };
  uint32_t reason_mask{ HealthReason::NONE };
};

// HealthContext is a read-only snapshot of FC facts. Monitors diagnose these
// facts but never change flight state or actuator output.
struct HealthContext
{
  uint32_t now_ms{ 0U };
  bool ready_to_arm{ false };
  bool armed{ false };
  bool failsafe_active{ false };
  bool terminated{ false };
  bool in_flight{ false };

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

  uint8_t ros_link_state{ HealthLinkState::UNKNOWN };
  uint8_t network_link_state{ HealthLinkState::UNKNOWN };
  uint8_t rc_link_state{ HealthLinkState::DISCONNECTED };
};

struct HealthComponentStatus
{
  uint8_t state{ HealthState::UNKNOWN };
  uint32_t reason_mask{ HealthReason::NONE };
  uint32_t last_change_ms{ 0U };
};

struct LinkHealthStatus
{
  HealthComponentStatus summary{};
  uint8_t ros{ 0U };
  uint8_t network{ 0U };
  uint8_t rc{ 0U };
};

struct FlightHealthReport
{
  HealthComponentStatus overall{};
  HealthComponentStatus flight{};
  LinkHealthStatus link{};
  HealthComponentStatus power{};
  HealthComponentStatus sensor{};
  HealthComponentStatus estimation{};
  HealthComponentStatus actuator{};
  HealthComponentStatus compute{};
  HealthComponentStatus config{};

  uint8_t requested_action{ HealthAction::NONE };
  uint8_t action_source{ static_cast<uint8_t>(HealthComponent::FLIGHT) };
  uint32_t action_reason_mask{ HealthReason::NONE };
  uint32_t action_since_ms{ 0U };
  float battery_voltage{ -1.0f };
  float battery_compensated_voltage{ -1.0f };
  float battery_percentage{ -1.0f };
  uint8_t estimation_validity{ 0U };
  uint32_t rejected_measurements{ 0U };
};
