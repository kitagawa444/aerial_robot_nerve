#pragma once

#include "flight_control/flight_control.h"

#ifdef SIMULATION

#include <spinal_msgs/msg/desire_coord.hpp>
#include <spinal_msgs/msg/flight_status.hpp>
#include <spinal_msgs/msg/flight_parameter_table.hpp>
#include <spinal_msgs/msg/health_config.hpp>
#include <spinal_msgs/msg/four_axis_command.hpp>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.hpp>
#include <spinal_msgs/msg/position_control_config.hpp>
#include <spinal_msgs/msg/position_control_setpoint.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_term.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.hpp>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.hpp>

#else

#include <spinal_msgs/msg/desire_coord.h>
#include <spinal_msgs/msg/flight_status.h>
#include <spinal_msgs/msg/flight_parameter_table.h>
#include <spinal_msgs/msg/health_config.h>
#include <spinal_msgs/msg/four_axis_command.h>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.h>
#include <spinal_msgs/msg/position_control_config.h>
#include <spinal_msgs/msg/position_control_setpoint.h>
#include <spinal_msgs/msg/roll_pitch_yaw_term.h>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.h>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.h>

#endif

namespace flight_control_ros
{

#ifdef SIMULATION
using FourAxisCommandMsg = spinal_msgs::msg::FourAxisCommand;
using RollPitchYawTermMsg = spinal_msgs::msg::RollPitchYawTerm;
using RollPitchYawTermsMsg = spinal_msgs::msg::RollPitchYawTerms;
using PMatrixMsg = spinal_msgs::msg::PMatrixPseudoInverseWithInertia;
using TorqueAllocationMsg = spinal_msgs::msg::TorqueAllocationMatrixInv;
using DesireCoordMsg = spinal_msgs::msg::DesireCoord;
using PositionControlConfigMsg = spinal_msgs::msg::PositionControlConfig;
using PositionControlSetpointMsg = spinal_msgs::msg::PositionControlSetpoint;
using FlightStatusMsg = spinal_msgs::msg::FlightStatus;
using HealthConfigMsg = spinal_msgs::msg::HealthConfig;
using FlightParameterTableMsg = spinal_msgs::msg::FlightParameterTable;
#else
using FourAxisCommandMsg = spinal_msgs__msg__FourAxisCommand;
using RollPitchYawTermMsg = spinal_msgs__msg__RollPitchYawTerm;
using RollPitchYawTermsMsg = spinal_msgs__msg__RollPitchYawTerms;
using PMatrixMsg = spinal_msgs__msg__PMatrixPseudoInverseWithInertia;
using TorqueAllocationMsg = spinal_msgs__msg__TorqueAllocationMatrixInv;
using DesireCoordMsg = spinal_msgs__msg__DesireCoord;
using PositionControlConfigMsg = spinal_msgs__msg__PositionControlConfig;
using PositionControlSetpointMsg = spinal_msgs__msg__PositionControlSetpoint;
using FlightStatusMsg = spinal_msgs__msg__FlightStatus;
using HealthConfigMsg = spinal_msgs__msg__HealthConfig;
using FlightParameterTableMsg = spinal_msgs__msg__FlightParameterTable;
#endif

bool applyFourAxisCommand(FlightControl &flight_control, const FourAxisCommandMsg &msg);
bool applyRpyGains(FlightControl &flight_control, const RollPitchYawTermsMsg &msg);
bool applyPMatrixInertia(FlightControl &flight_control, const PMatrixMsg &msg);
bool applyTorqueAllocationMatrixInv(FlightControl &flight_control, const TorqueAllocationMsg &msg);
void applyOffsetRotation(FlightControl &flight_control, const DesireCoordMsg &msg);
bool applyPositionControlConfig(FlightControl &flight_control, const PositionControlConfigMsg &msg);
bool applyHealthConfig(FlightControl &flight_control, const HealthConfigMsg &msg);
bool applyPositionControlSetpoint(FlightControl &flight_control, const PositionControlSetpointMsg &msg);

void fillRollPitchYawTerm(RollPitchYawTermMsg &msg, const FlightControlRpyTerm &src);
void fillFlightStatus(FlightStatusMsg &msg, const FlightSupervisorStatus &src);
void fillFlightParameterTable(FlightParameterTableMsg &msg, const FlightParameterDatabase &database, bool applied,
                              bool persistent_storage);
void fillFlightParameterTable(FlightParameterTableMsg &msg, const FlightParameterPayload &payload, bool valid,
                              bool dirty, bool applied, bool persistent_storage, uint16_t schema_version,
                              uint32_t generation, uint32_t crc32, uint32_t valid_fields);

}  // namespace flight_control_ros
