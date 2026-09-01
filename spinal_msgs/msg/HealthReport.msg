spinal_msgs/HealthStatus overall
spinal_msgs/HealthStatus flight
spinal_msgs/LinkHealth link
spinal_msgs/HealthStatus power
spinal_msgs/HealthStatus sensor
spinal_msgs/HealthStatus estimation
spinal_msgs/HealthStatus actuator
spinal_msgs/HealthStatus compute
spinal_msgs/HealthStatus config

uint8 ACTION_NONE = 0
uint8 ACTION_INHIBIT_ARM = 1
uint8 ACTION_FORCE_LAND = 2
uint8 ACTION_TERMINATE = 3

uint8 COMPONENT_FLIGHT = 0
uint8 COMPONENT_LINK = 1
uint8 COMPONENT_POWER = 2
uint8 COMPONENT_SENSOR = 3
uint8 COMPONENT_ESTIMATION = 4
uint8 COMPONENT_ACTUATOR = 5
uint8 COMPONENT_COMPUTE = 6
uint8 COMPONENT_CONFIG = 7

uint8 requested_action
uint8 action_source
uint32 action_reason_mask
uint32 action_since_ms

float32 battery_voltage
float32 battery_compensated_voltage
float32 battery_percentage
uint8 estimation_validity
uint32 rejected_measurements
