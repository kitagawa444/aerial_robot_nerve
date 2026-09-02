#include "flight_control/supervisor/flight_supervisor.h"

#include <cmath>

#include "flight_control/flight_control_types.h"

void FlightSupervisor::reset() {
  status_ = FlightSupervisorStatus();
  stable_since_ms_ = 0U;
  last_rc_active_ms_ = 0U;
}

bool FlightSupervisor::configure(const FlightSupervisorConfig &config) {
  if (!finite_(config.takeoff_height) || config.takeoff_height <= 0.0f ||
      !finite_(config.takeoff_position_tolerance) ||
      config.takeoff_position_tolerance <= 0.0f ||
      !finite_(config.takeoff_velocity_tolerance) ||
      config.takeoff_velocity_tolerance <= 0.0f ||
      config.takeoff_stable_time_ms == 0U || !finite_(config.landing_speed) ||
      config.landing_speed >= 0.0f || !finite_(config.landed_height) ||
      config.landed_height < 0.0f || !finite_(config.landed_velocity) ||
      config.landed_velocity <= 0.0f || config.landed_stable_time_ms == 0U ||
      config.rc_authority_timeout_ms == 0U) {
    return false;
  }
  config_ = config;
  return true;
}

bool FlightSupervisor::configureHealth(const HealthManagerConfig &config) {
  return health_manager_.configure(config);
}

bool FlightSupervisor::request(uint8_t command, uint8_t source,
                               const FlightSupervisorInput &input) {
  uint8_t rejection = FlightCommandResult::ACCEPTED;
  updateHealth_(input);

  switch (command) {
  case FlightControlCommand::ARM_ON_CMD:
    if (status_.arming_state == FlightArmingState::ARMED) {
      rejection = FlightCommandResult::ALREADY_IN_STATE;
      break;
    }
    if (!input.attitude_ready) {
      rejection = FlightCommandResult::ATTITUDE_NOT_READY;
      break;
    }
    if (input.position_control_enabled && !input.position_ready) {
      rejection = FlightCommandResult::POSITION_NOT_READY;
      break;
    }
    if (status_.health.requested_action == HealthAction::INHIBIT_ARM) {
      rejection = FlightCommandResult::HEALTH_INHIBIT;
      break;
    }
    status_.takeoff_reference_height = input.vertical_position;
    status_.takeoff_target_height =
        input.vertical_position + config_.takeoff_height;
    status_.user_intention = input.position_control_enabled
                                 ? FlightControlMode::POSITION
                                 : FlightControlMode::ATTITUDE;
    stable_since_ms_ = 0U;
    transition_(FlightArmingState::ARMED, FlightPhase::LANDED,
                input.position_control_enabled ? FlightControlMode::POSITION
                                               : FlightControlMode::ATTITUDE,
                input.position_control_enabled
                    ? static_cast<uint8_t>(FlightCommandSource::INTERNAL)
                    : source,
                FlightFailsafeState::NONE,
                FlightTransitionReason::COMMAND_ACCEPTED);
    break;

  case FlightControlCommand::ARM_OFF_CMD:
    if (status_.arming_state == FlightArmingState::DISARMED) {
      rejection = FlightCommandResult::ALREADY_IN_STATE;
    } else if (status_.flight_phase != FlightPhase::LANDED) {
      rejection = FlightCommandResult::IN_AIR;
    } else {
      status_.user_intention = FlightControlMode::IDLE;
      disarm_(FlightControlMode::IDLE, FlightFailsafeState::NONE,
              FlightTransitionReason::COMMAND_ACCEPTED);
    }
    break;

  case FlightControlCommand::TAKEOFF_CMD:
    if (status_.arming_state != FlightArmingState::ARMED) {
      rejection = FlightCommandResult::NOT_ARMED;
    } else if (status_.flight_phase != FlightPhase::LANDED) {
      rejection = FlightCommandResult::NOT_LANDED;
    } else if (!input.position_control_enabled || !input.position_ready) {
      rejection = FlightCommandResult::POSITION_NOT_READY;
    } else {
      status_.takeoff_reference_height = input.vertical_position;
      status_.takeoff_target_height =
          input.vertical_position + config_.takeoff_height;
      status_.user_intention = FlightControlMode::TAKEOFF;
      stable_since_ms_ = 0U;
      transition_(FlightArmingState::ARMED, FlightPhase::TAKING_OFF,
                  FlightControlMode::TAKEOFF, FlightCommandSource::INTERNAL,
                  FlightFailsafeState::NONE,
                  FlightTransitionReason::COMMAND_ACCEPTED);
    }
    break;

  case FlightControlCommand::LAND_CMD:
    if (status_.arming_state != FlightArmingState::ARMED) {
      rejection = FlightCommandResult::NOT_ARMED;
    } else if (status_.flight_phase == FlightPhase::LANDING) {
      rejection = FlightCommandResult::ALREADY_IN_STATE;
    } else if (status_.flight_phase == FlightPhase::LANDED) {
      rejection = FlightCommandResult::ALREADY_IN_STATE;
    } else if (!input.position_control_enabled || !input.position_ready) {
      rejection = FlightCommandResult::POSITION_NOT_READY;
    } else {
      status_.user_intention = FlightControlMode::LAND;
      stable_since_ms_ = 0U;
      transition_(FlightArmingState::ARMED, FlightPhase::LANDING,
                  FlightControlMode::LAND, FlightCommandSource::INTERNAL,
                  FlightFailsafeState::NONE,
                  FlightTransitionReason::COMMAND_ACCEPTED);
    }
    break;

  case FlightControlCommand::FORCE_LANDING_CMD:
    if (status_.arming_state != FlightArmingState::ARMED) {
      rejection = FlightCommandResult::NOT_ARMED;
    } else {
      status_.user_intention = FlightControlMode::FORCE_LAND;
      enterForceLand_(FlightTransitionReason::COMMAND_ACCEPTED);
    }
    break;

  case FlightControlCommand::HALT_CMD:
    status_.user_intention = FlightControlMode::TERMINATED;
    disarm_(FlightControlMode::TERMINATED, FlightFailsafeState::TERMINATED,
            FlightTransitionReason::COMMAND_ACCEPTED);
    break;

  default:
    rejection = FlightCommandResult::UNSUPPORTED;
    break;
  }

  setCommandResult_(command, source, rejection);
  updateHealth_(input);
  return rejection == FlightCommandResult::ACCEPTED;
}

void FlightSupervisor::update(const FlightSupervisorInput &input) {
  updateHealth_(input);
  if (applyHealthAction_()) {
    updateHealth_(input);
    return;
  }

  if (input.rc_active) {
    last_rc_active_ms_ = input.now_ms;
    if (status_.arming_state == FlightArmingState::ARMED &&
        status_.flight_phase == FlightPhase::AIRBORNE &&
        status_.control_mode == FlightControlMode::POSITION &&
        status_.failsafe == FlightFailsafeState::NONE) {
      if (status_.authority != FlightCommandSource::RC) {
        status_.authority = FlightCommandSource::RC;
        ++status_.sequence;
      }
    }
  } else if (status_.authority == FlightCommandSource::RC &&
             static_cast<uint32_t>(input.now_ms - last_rc_active_ms_) >
                 config_.rc_authority_timeout_ms) {
    status_.authority = FlightCommandSource::INTERNAL;
    ++status_.sequence;
  }

  if (status_.arming_state != FlightArmingState::ARMED) {
    updateHealth_(input);
    return;
  }

  const bool position_mode =
      status_.control_mode == FlightControlMode::POSITION ||
      status_.control_mode == FlightControlMode::TAKEOFF ||
      status_.control_mode == FlightControlMode::LAND;
  if (position_mode && !input.position_ready) {
    status_.user_intention = FlightControlMode::FORCE_LAND;
    enterForceLand_(FlightTransitionReason::POSITION_LOST);
    updateHealth_(input);
    return;
  }

  if (status_.control_mode == FlightControlMode::TAKEOFF) {
    const bool converged =
        std::fabs(input.vertical_position - status_.takeoff_target_height) <=
            config_.takeoff_position_tolerance &&
        std::fabs(input.vertical_velocity) <=
            config_.takeoff_velocity_tolerance;
    if (!converged) {
      stable_since_ms_ = 0U;
    } else if (stable_since_ms_ == 0U) {
      stable_since_ms_ = input.now_ms == 0U ? 1U : input.now_ms;
    } else if (static_cast<uint32_t>(input.now_ms - stable_since_ms_) >=
               config_.takeoff_stable_time_ms) {
      stable_since_ms_ = 0U;
      status_.user_intention = FlightControlMode::POSITION;
      transition_(FlightArmingState::ARMED, FlightPhase::AIRBORNE,
                  FlightControlMode::POSITION, status_.authority,
                  FlightFailsafeState::NONE,
                  FlightTransitionReason::TAKEOFF_COMPLETE);
    }
  } else if (status_.control_mode == FlightControlMode::LAND) {
    const bool landed =
        input.vertical_position <=
            status_.takeoff_reference_height + config_.landed_height &&
        std::fabs(input.vertical_velocity) <= config_.landed_velocity;
    if (!landed) {
      stable_since_ms_ = 0U;
    } else if (stable_since_ms_ == 0U) {
      stable_since_ms_ = input.now_ms == 0U ? 1U : input.now_ms;
    } else if (static_cast<uint32_t>(input.now_ms - stable_since_ms_) >=
               config_.landed_stable_time_ms) {
      stable_since_ms_ = 0U;
      status_.user_intention = FlightControlMode::IDLE;
      disarm_(FlightControlMode::IDLE, FlightFailsafeState::NONE,
              FlightTransitionReason::LANDING_COMPLETE);
    }
  }
  updateHealth_(input);
}

void FlightSupervisor::reportControllerForceLand(
    const FlightSupervisorInput &input) {
  if (status_.arming_state == FlightArmingState::ARMED &&
      status_.failsafe == FlightFailsafeState::NONE) {
    status_.user_intention = FlightControlMode::FORCE_LAND;
    enterForceLand_(FlightTransitionReason::CONTROLLER_REPORTED_FORCE_LAND);
  }
  updateHealth_(input);
}

void FlightSupervisor::noteExternalSetpoint(uint8_t source) {
  if (status_.arming_state != FlightArmingState::ARMED ||
      status_.flight_phase != FlightPhase::AIRBORNE ||
      status_.control_mode != FlightControlMode::POSITION ||
      status_.failsafe != FlightFailsafeState::NONE ||
      status_.authority == FlightCommandSource::RC) {
    return;
  }
  if (status_.authority != source) {
    status_.authority = source;
    ++status_.sequence;
  }
}

void FlightSupervisor::updateHealth_(const FlightSupervisorInput &input) {
  const bool ready_to_arm =
      input.attitude_ready && input.estimation_ready && input.actuator_ready &&
      input.config_ready &&
      (!input.position_control_enabled || input.position_ready);
  const bool previous_ready_to_arm = status_.ready_to_arm;
  bool health_changed =
      status_.attitude_ready != input.attitude_ready ||
      status_.position_ready != input.position_ready ||
      status_.rc_connected != input.rc_connected ||
      status_.ros_link_state != input.ros_link_state ||
      status_.network_link_state != input.network_link_state ||
      status_.rc_link_state != input.rc_link_state;

  status_.attitude_ready = input.attitude_ready;
  status_.position_ready = input.position_ready;
  status_.rc_connected = input.rc_connected;
  status_.ros_link_state = input.ros_link_state;
  status_.network_link_state = input.network_link_state;
  status_.rc_link_state = input.rc_link_state;

  HealthContext health_context;
  health_context.now_ms = input.now_ms;
  health_context.ready_to_arm = ready_to_arm;
  health_context.armed = status_.arming_state == FlightArmingState::ARMED;
  health_context.failsafe_active =
      status_.failsafe != FlightFailsafeState::NONE;
  health_context.terminated =
      status_.failsafe == FlightFailsafeState::TERMINATED;
  health_context.in_flight = status_.flight_phase != FlightPhase::LANDED;
  health_context.estimation_ready = input.estimation_ready;
  health_context.estimation_validity = input.estimation_validity;
  health_context.rejected_measurements = input.rejected_measurements;
  health_context.actuator_ready = input.actuator_ready;
  health_context.config_ready = input.config_ready;
  health_context.power_monitor_present = input.power_monitor_present;
  health_context.battery_voltage = input.battery_voltage;
  health_context.primary_imu_present = input.primary_imu_present;
  health_context.last_primary_imu_update_ms = input.last_primary_imu_update_ms;
  health_context.control_loop_period_valid = input.control_loop_period_valid;
  health_context.control_loop_period_ms = input.control_loop_period_ms;
  health_context.control_loop_sequence = input.control_loop_sequence;
  health_context.ros_link_state = input.ros_link_state;
  health_context.network_link_state = input.network_link_state;
  health_context.rc_link_state = input.rc_link_state;
  health_changed |= health_manager_.update(health_context, status_.health);
  status_.ready_to_arm = ready_to_arm && status_.health.requested_action !=
                                             HealthAction::INHIBIT_ARM;
  health_changed |= previous_ready_to_arm != status_.ready_to_arm;

  if (health_changed)
    ++status_.sequence;
}

bool FlightSupervisor::applyHealthAction_() {
  if (status_.arming_state != FlightArmingState::ARMED)
    return false;

  if (status_.health.requested_action == HealthAction::TERMINATE) {
    status_.user_intention = FlightControlMode::TERMINATED;
    disarm_(FlightControlMode::TERMINATED, FlightFailsafeState::TERMINATED,
            FlightTransitionReason::HEALTH_TERMINATE);
    return true;
  }
  if (status_.health.requested_action == HealthAction::FORCE_LAND &&
      status_.failsafe == FlightFailsafeState::NONE) {
    status_.user_intention = FlightControlMode::FORCE_LAND;
    enterForceLand_(FlightTransitionReason::HEALTH_FORCE_LAND);
    return true;
  }
  return false;
}

void FlightSupervisor::setCommandResult_(uint8_t command, uint8_t source,
                                         uint8_t result) {
  ++status_.command_sequence;
  status_.last_command = command;
  status_.last_command_source = source;
  status_.last_command_result = result;
  ++status_.sequence;
}

void FlightSupervisor::transition_(uint8_t arming_state, uint8_t flight_phase,
                                   uint8_t control_mode, uint8_t authority,
                                   uint8_t failsafe, uint8_t reason) {
  status_.arming_state = arming_state;
  status_.flight_phase = flight_phase;
  status_.control_mode = control_mode;
  status_.authority = authority;
  status_.failsafe = failsafe;
  status_.transition_reason = reason;
  ++status_.sequence;
}

void FlightSupervisor::enterForceLand_(uint8_t reason) {
  stable_since_ms_ = 0U;
  transition_(FlightArmingState::ARMED, FlightPhase::LANDING,
              FlightControlMode::FORCE_LAND, FlightCommandSource::FAILSAFE,
              FlightFailsafeState::FORCE_LAND, reason);
}

void FlightSupervisor::disarm_(uint8_t mode, uint8_t failsafe, uint8_t reason) {
  stable_since_ms_ = 0U;
  transition_(FlightArmingState::DISARMED, FlightPhase::LANDED, mode,
              FlightCommandSource::NONE, failsafe, reason);
}

bool FlightSupervisor::finite_(float value) { return std::isfinite(value); }
