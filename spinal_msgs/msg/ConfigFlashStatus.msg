uint8 UART3_DISABLED = 0
uint8 UART3_GPS = 1
uint8 UART3_CRSF = 2

uint8 IMU_DISABLED = 0
uint8 IMU_MPU9250 = 1
uint8 IMU_ICM20948 = 2

uint8 SERVO_DISABLED = 0
uint8 SERVO_DYNAMIXEL = 1
uint8 SERVO_KONDO = 2

uint8 MOTOR_OUTPUT_DISABLED = 0
uint8 MOTOR_OUTPUT_PWM = 1
uint8 MOTOR_OUTPUT_DSHOT = 2

bool valid
bool dirty
bool reboot_required
bool persistent_storage
int8 active_slot
uint8 active_uart3_driver
uint8 pending_uart3_driver
uint8 active_imu_driver
uint8 pending_imu_driver
bool active_barometer_enabled
bool pending_barometer_enabled
uint8 active_servo_driver
uint8 pending_servo_driver
bool active_attitude_estimation_enabled
bool pending_attitude_estimation_enabled
bool active_height_estimation_enabled
bool pending_height_estimation_enabled
bool active_position_estimation_enabled
bool pending_position_estimation_enabled
bool active_flight_control_enabled
bool pending_flight_control_enabled
uint8 active_motor_output_driver
uint8 pending_motor_output_driver
uint16 schema_version
uint32 generation
uint32 crc32
