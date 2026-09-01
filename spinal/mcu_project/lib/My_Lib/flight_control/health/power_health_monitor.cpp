#include "flight_control/health/power_health_monitor.h"

#include <cstddef>
#include <cmath>

namespace
{
constexpr float VOLTAGE_TABLE[] = { 3.209f, 3.683f, 3.747f, 3.791f, 3.812f, 3.839f,
                                    3.883f, 3.936f, 3.999f, 4.085f, 4.200f };
constexpr std::size_t VOLTAGE_TABLE_SIZE = sizeof(VOLTAGE_TABLE) / sizeof(VOLTAGE_TABLE[0]);
}

bool PowerHealthMonitor::configure(const PowerHealthConfig &config)
{
  if (config.battery_cell_count == 0U || !std::isfinite(config.low_percentage) || config.low_percentage < 0.0f ||
      config.low_percentage > 100.0f || !std::isfinite(config.hysteresis_percentage) ||
      config.hysteresis_percentage < 0.0f || !std::isfinite(config.high_cell_threshold) ||
      config.high_cell_threshold < 0.0f || !std::isfinite(config.resistance) || config.resistance < 0.0f ||
      !std::isfinite(config.resistance_voltage_rate) || !std::isfinite(config.hovering_current) ||
      config.hovering_current < 0.0f || config.debounce_ms == 0U)
  {
    return false;
  }
  config_ = config;
  configured_ = true;
  low_voltage_latched_ = false;
  low_voltage_since_ms_ = 0U;
  return true;
}

HealthEvaluation PowerHealthMonitor::evaluate(const HealthContext &context)
{
  compensated_voltage_ = context.battery_voltage;
  battery_percentage_ = -1.0f;
  if (!context.power_monitor_present || !std::isfinite(context.battery_voltage) || context.battery_voltage <= 0.0f)
  {
    low_voltage_latched_ = false;
    low_voltage_since_ms_ = 0U;
    return { HealthState::UNKNOWN, HealthReason::DATA_UNAVAILABLE };
  }
  if (!configured_)
  {
    return { HealthState::DEGRADED, HealthReason::THRESHOLD_UNCONFIGURED };
  }

  if (context.in_flight)
  {
    compensated_voltage_ += (config_.resistance_voltage_rate * compensated_voltage_ + config_.resistance) *
                            config_.hovering_current;
  }

  const float estimated_cell_count = compensated_voltage_ / VOLTAGE_TABLE[VOLTAGE_TABLE_SIZE - 1U];
  if (estimated_cell_count - static_cast<float>(config_.battery_cell_count) > config_.high_cell_threshold)
  {
    low_voltage_latched_ = false;
    low_voltage_since_ms_ = 0U;
    return { HealthState::CRITICAL, HealthReason::BATTERY_CELL_MISMATCH };
  }

  battery_percentage_ = percentageFromCellVoltage_(compensated_voltage_ /
                                                   static_cast<float>(config_.battery_cell_count));
  if (battery_percentage_ < 0.0f)
  {
    low_voltage_latched_ = false;
    low_voltage_since_ms_ = 0U;
    return { HealthState::DEGRADED, HealthReason::INVALID };
  }

  const bool below_threshold = battery_percentage_ < config_.low_percentage;
  const bool recovered = battery_percentage_ >= config_.low_percentage + config_.hysteresis_percentage;
  if (low_voltage_latched_)
  {
    if (!recovered) return { HealthState::CRITICAL, HealthReason::LOW_VOLTAGE };
    low_voltage_latched_ = false;
    low_voltage_since_ms_ = 0U;
  }

  if (!below_threshold)
  {
    low_voltage_since_ms_ = 0U;
    return { HealthState::OK, HealthReason::NONE };
  }

  if (low_voltage_since_ms_ == 0U) low_voltage_since_ms_ = context.now_ms == 0U ? 1U : context.now_ms;
  if (static_cast<uint32_t>(context.now_ms - low_voltage_since_ms_) < config_.debounce_ms)
  {
    return { HealthState::DEGRADED, HealthReason::LOW_VOLTAGE };
  }

  low_voltage_latched_ = true;
  return { HealthState::CRITICAL, HealthReason::LOW_VOLTAGE };
}

float PowerHealthMonitor::percentageFromCellVoltage_(float voltage)
{
  if (!std::isfinite(voltage) || voltage < VOLTAGE_TABLE[0]) return -1.0f;
  if (voltage >= VOLTAGE_TABLE[VOLTAGE_TABLE_SIZE - 1U]) return 100.0f;

  for (std::size_t index = 1U; index < VOLTAGE_TABLE_SIZE; ++index)
  {
    if (voltage <= VOLTAGE_TABLE[index])
    {
      const float lower = VOLTAGE_TABLE[index - 1U];
      const float upper = VOLTAGE_TABLE[index];
      return static_cast<float>(index - 1U) * 10.0f + (voltage - lower) / (upper - lower) * 10.0f;
    }
  }
  return 100.0f;
}
