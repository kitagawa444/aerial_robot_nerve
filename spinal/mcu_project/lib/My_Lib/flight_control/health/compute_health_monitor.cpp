#include "flight_control/health/compute_health_monitor.h"

bool ComputeHealthMonitor::configure(const ComputeHealthConfig &config)
{
  if (config.control_loop_deadline_ms == 0U || config.miss_limit == 0U) return false;
  config_ = config;
  consecutive_misses_ = 0U;
  last_sequence_ = 0U;
  last_evaluation_ = HealthEvaluation();
  return true;
}

HealthEvaluation ComputeHealthMonitor::evaluate(const HealthContext &context)
{
  if (!context.control_loop_period_valid) return { HealthState::UNKNOWN, HealthReason::DATA_UNAVAILABLE };
  if (context.control_loop_sequence == last_sequence_) return last_evaluation_;
  last_sequence_ = context.control_loop_sequence;

  if (context.control_loop_period_ms <= config_.control_loop_deadline_ms)
  {
    consecutive_misses_ = 0U;
    last_evaluation_ = { HealthState::OK, HealthReason::NONE };
    return last_evaluation_;
  }

  if (consecutive_misses_ < config_.miss_limit) ++consecutive_misses_;
  last_evaluation_ = { consecutive_misses_ >= config_.miss_limit ? HealthState::CRITICAL : HealthState::DEGRADED,
                       HealthReason::DEADLINE_MISSED };
  return last_evaluation_;
}
