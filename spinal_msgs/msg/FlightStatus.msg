builtin_interfaces/Time stamp

uint8 ARMING_DISARMED = 0
uint8 ARMING_ARMED = 1

uint8 PHASE_LANDED = 0
uint8 PHASE_TAKING_OFF = 1
uint8 PHASE_AIRBORNE = 2
uint8 PHASE_LANDING = 3

uint8 MODE_IDLE = 0
uint8 MODE_ATTITUDE = 1
uint8 MODE_POSITION = 2
uint8 MODE_TAKEOFF = 3
uint8 MODE_LAND = 4
uint8 MODE_FORCE_LAND = 5
uint8 MODE_TERMINATED = 6

uint8 AUTHORITY_NONE = 0
uint8 AUTHORITY_ROS = 1
uint8 AUTHORITY_RC = 2
uint8 AUTHORITY_INTERNAL = 3
uint8 AUTHORITY_FAILSAFE = 4

uint8 FAILSAFE_NONE = 0
uint8 FAILSAFE_FORCE_LAND = 1
uint8 FAILSAFE_TERMINATED = 2

uint8 RESULT_ACCEPTED = 0
uint8 RESULT_UNSUPPORTED = 1
uint8 RESULT_ALREADY_IN_STATE = 2
uint8 RESULT_NOT_ARMED = 3
uint8 RESULT_NOT_LANDED = 4
uint8 RESULT_ATTITUDE_NOT_READY = 5
uint8 RESULT_POSITION_NOT_READY = 6
uint8 RESULT_IN_AIR = 7
uint8 RESULT_HEALTH_INHIBIT = 8

uint8 TRANSITION_NONE = 0
uint8 TRANSITION_COMMAND_ACCEPTED = 1
uint8 TRANSITION_TAKEOFF_COMPLETE = 2
uint8 TRANSITION_LANDING_COMPLETE = 3
uint8 TRANSITION_POSITION_LOST = 4
uint8 TRANSITION_CONTROLLER_FORCE_LAND = 5
uint8 TRANSITION_HEALTH_FORCE_LAND = 6
uint8 TRANSITION_HEALTH_TERMINATE = 7

uint8 LINK_UNKNOWN = 0
uint8 LINK_DISCONNECTED = 1
uint8 LINK_CONNECTING = 2
uint8 LINK_CONNECTED = 3
uint8 LINK_STALE = 4
uint8 LINK_DISABLED = 5

uint32 sequence
uint32 command_sequence
uint8 arming_state
uint8 flight_phase
uint8 control_mode
uint8 user_intention
uint8 authority
uint8 failsafe
uint8 last_command
uint8 last_command_source
uint8 last_command_result
uint8 transition_reason
bool ready_to_arm
bool attitude_ready
bool position_ready
bool rc_connected
uint8 ros_link_state
uint8 network_link_state
uint8 rc_link_state
spinal_msgs/HealthReport health
float32 takeoff_reference_height
float32 takeoff_target_height
