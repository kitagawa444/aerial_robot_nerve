builtin_interfaces/Time stamp

uint8 POSITION = 1
uint8 VELOCITY = 2
uint8 ATTITUDE = 4

uint8 SPINAL_MODE_EGOMOTION = 3
uint8 SPINAL_MODE_EXPERIMENT = 4
uint8 SPINAL_MODE_GROUND_TRUTH = 5

uint8 estimate_mode
uint8 field_mask
float32[3] position
float32[3] position_variance
float32[3] velocity
float32[3] velocity_variance
float32[4] attitude  # Quaternion [x, y, z, w]
float32[3] attitude_variance
float32[3] angular_velocity
