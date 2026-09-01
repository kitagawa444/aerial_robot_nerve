/*
******************************************************************************
* File Name          : config.h
* Description        : Basic configuration for this board(c.f. GPIO Macro, Enable flag for different processes
******************************************************************************
*/
#ifndef __CONFIG_H
#define __CONFIG_H
#define SPINAL

#if defined(STM32F756xx) || defined(STM32F746xx) || defined(STM32F745xx) || defined(STM32F765xx) ||                    \
    defined(STM32F767xx) || defined(STM32F769xx) || defined(STM32F777xx) || defined(STM32F779xx) ||                    \
    defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F732xx) || defined(STM32F733xx) ||                    \
    defined(STM32F730xx) || defined(STM32F750xx)
#include "stm32f7xx_hal.h"
#define STM32F7
#endif

#if defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) ||                    \
    defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) ||                    \
    defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H7B3xx) || defined(STM32H7B3xxQ) ||                  \
    defined(STM32H7B0xx) || defined(STM32H7B0xxQ) || defined(STM32H735xx) || defined(STM32H733xx) ||                   \
    defined(STM32H730xx) || defined(STM32H730xxQ) || defined(STM32H725xx) || defined(STM32H723xx)
#include "stm32h7xx_hal.h"
#define STM32H7
#endif

// define function
#define GPIO_H(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define GPIO_L(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)

// 1. Specials board options
#define STM32H7_V2 1

// 2. Application capabilities. These flags only control which drivers are
// compiled into Application Flash. Config Flash selects the driver and the
// enabled subsystems at boot; changing a selection does not require rebuilding.
#define CAPABILITY_IMU_ICM20948 1
#define CAPABILITY_IMU_MPU9250 1
#define CAPABILITY_BAROMETER 1
#define CAPABILITY_GPS_UART3 1
#define CAPABILITY_CRSF_UART3 1
#define CAPABILITY_DYNAMIXEL 1
#define CAPABILITY_KONDO 1
#define CAPABILITY_PWM 1
#define CAPABILITY_DSHOT 1
#define CAPABILITY_ATTITUDE_ESTIMATION 1
#define CAPABILITY_HEIGHT_ESTIMATION 1
#define CAPABILITY_POSITION_ESTIMATION 1
#define CAPABILITY_FLIGHT_CONTROL 1

// 3. Defaults used only when neither Config Flash slot contains a valid image.
#define DEFAULT_IMU_DRIVER 2           // ImuDriver::ICM20948
#define DEFAULT_BAROMETER_ENABLED 1
#define DEFAULT_UART3_DRIVER 2         // Uart3Driver::CRSF
#define DEFAULT_SERVO_DRIVER 1         // ServoDriver::DYNAMIXEL
#define DEFAULT_ATTITUDE_ESTIMATION_ENABLED 1
#define DEFAULT_HEIGHT_ESTIMATION_ENABLED 1
#define DEFAULT_POSITION_ESTIMATION_ENABLED 1
#define DEFAULT_FLIGHT_CONTROL_ENABLED 1
#define DEFAULT_MOTOR_OUTPUT_DRIVER 2  // MotorOutputDriver::DSHOT

#endif  //__CONFIG_H
