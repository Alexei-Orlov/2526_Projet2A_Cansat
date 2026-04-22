
/*
 * imu.h
 *
 *  Created on: Jan 14, 2026
 *      Author: juanp
 */

/*
 * imu.h
 * BNO055 I2C Driver for STM32G4
 */

#ifndef IMU_H_
#define IMU_H_

// Changed to G4 HAL to match your STM32G431CBU6
#include "stm32g4xx_hal.h"

// I2C Address (AD0 = GND -> 0x28, AD0 = VCC -> 0x29)
#define BNO055_I2C_ADDR_LO      (0x28 << 1)
#define BNO055_I2C_ADDR_HI      (0x29 << 1)
#define BNO055_I2C_ADDR         BNO055_I2C_ADDR_HI


// Register Addresses
#define BNO055_CHIP_ID_ADDR     	0x00
#define BNO055_OPR_MODE_ADDR    	0x3D
#define BNO055_CALIB_STAT_ADDR  	0x35
#define BNO055_SYS_TRIGGER_ADDR 	0x3F
#define BNO055_EUL_HEADING_LSB  	0x1A // Start of Euler Data
#define BNO055_LIA_DATA_X_LSB   	0x28 // Start of Linear Acceleration Data
#define BNO055_SYS_CLK_STATUS_ADDR 	0x38
#define BNO055_ACCEL_OFFSET_X_LSB 	0x55 // Calibration profile for 1 time calib
#define BNO055_GYRO_DATA_X_LSB  	0x14

// Operation Modes
#define OPR_MODE_CONFIG         0x00
#define OPR_MODE_NDOF           0x0C

// Updated Structure to hold sensor data for RTOS queues
typedef struct {
    char timestamp[16];
    float head;
    float pitch;
    float roll;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint8_t sys_calib;
    uint8_t gyro_calib;
    uint8_t accel_calib;
    uint8_t mag_calib;
} IMU_Data_t;

// Function Prototypes
uint8_t IMU_Init(I2C_HandleTypeDef *hi2c);

// Pass a UART handle to see the calibration progress on your PC
uint8_t IMU_calibration(UART_HandleTypeDef *huart_debug);
void IMU_GetCalibrationProfile(uint8_t *profile);
void IMU_SetCalibrationProfile(const uint8_t *profile);

// Gets Euler and Linear Acceleration
uint8_t getOrientationIMU(IMU_Data_t *data);

#endif /* IMU_H_ */
