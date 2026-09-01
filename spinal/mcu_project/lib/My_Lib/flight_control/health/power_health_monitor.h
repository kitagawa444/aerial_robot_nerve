#pragma once

#include "flight_control/health/health_monitor.h"

class PowerHealthMonitor final : public HealthMonitor
{
public:
  HealthComponent component() const override { return HealthComponent::POWER; }
  bool configure(const PowerHealthConfig &config);
  HealthEvaluation evaluate(const HealthContext &context) override;
  float compensatedVoltage() const { return compensated_voltage_; }
  float batteryPercentage() const { return battery_percentage_; }

private:
  PowerHealthConfig config_{};
  bool configured_{ false };
  bool low_voltage_latched_{ false };
  uint32_t low_voltage_since_ms_{ 0U };
  float compensated_voltage_{ -1.0f };
  float battery_percentage_{ -1.0f };

  static float percentageFromCellVoltage_(float voltage);
};
