builtin_interfaces/Time stamp

uint8 ATTITUDE_VALID = 1
uint8 VERTICAL_POSITION_VALID = 2
uint8 HORIZONTAL_POSITION_VALID = 4
uint8 VELOCITY_VALID = 8

uint8 validity
float32[3] position
float32[3] velocity
float32[3] acceleration
float32[4] attitude  # Quaternion [x, y, z, w]
float32[3] angular_velocity  # Bias-corrected body rate in the CoG frame
float32[3] accelerometer_bias
float32[3] gyroscope_bias
float32[15] covariance_diagonal
uint32 rejected_measurements
