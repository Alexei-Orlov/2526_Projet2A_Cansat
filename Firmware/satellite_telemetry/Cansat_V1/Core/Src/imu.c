/*
 * imu.c
 *
 * Created on: Jan 14, 2026
 * Author: juanp
 */

#include "imu.h"
#include <string.h>
#include <stdio.h>

static I2C_HandleTypeDef *imu_i2c;

const uint8_t imu_calib_profile[22] = {
    0x01, 0x00, 0xFD, 0xFF, 0xFD, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00,
    0x00, 0x00, 0xE8, 0x03, 0x00, 0x00
};

// Helper to write to a register
static void BNO_WriteReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(imu_i2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

// Helper to read from a register
static void BNO_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10);
}

uint8_t IMU_Init(I2C_HandleTypeDef *hi2c) {
    imu_i2c = hi2c;
    uint8_t id = 0;

    // 1. Check communication and Chip ID
    BNO_ReadRegs(BNO055_CHIP_ID_ADDR, &id, 1);
    if (id != 0xA0) {
        return 0; // Error: Device not found
    }

    // 2. Reset Mode to CONFIG
    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_CONFIG);
    HAL_Delay(30);

    // 3. Set INTERNAL Crystal
    BNO_WriteReg(BNO055_SYS_TRIGGER_ADDR, 0x00);
    HAL_Delay(50);

    // --- 4. INJECT THE HARDCODED CALIBRATION PROFILE ---
    IMU_SetCalibrationProfile(imu_calib_profile);
    HAL_Delay(30);

    // 5. Set Operation Mode to NDOF
    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_NDOF);
    HAL_Delay(30);

    return 1;
}

/**
 * @brief Extracts the 22 bytes of calibration data.
 */
void IMU_GetCalibrationProfile(uint8_t *profile) {
    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_CONFIG);
    HAL_Delay(25);
    BNO_ReadRegs(BNO055_ACCEL_OFFSET_X_LSB, profile, 22);
    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_NDOF);
    HAL_Delay(25);
}

/**
 * @brief Injects the 22 bytes of calibration data.
 */
void IMU_SetCalibrationProfile(const uint8_t *profile) {
    for(int i = 0; i < 22; i++) {
        BNO_WriteReg(BNO055_ACCEL_OFFSET_X_LSB + i, profile[i]);
        HAL_Delay(2);
    }
}

/**
 * @brief Blocking function to calibrate the IMU.
 */
uint8_t IMU_calibration(UART_HandleTypeDef *huart_debug) {
    uint8_t calib = 0;
    uint8_t sys=0, gyro=0, accel=0, mag=0;
    char msg[128];

    sprintf(msg, "\r\n[*] IMU Calibration Started. Follow instructions:\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "1. Gyro:  Leave sensor perfectly still.\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "2. Mag:   Move sensor in a Figure-8 pattern.\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "3. Accel: Place resting on each of its 6 sides for 3 seconds.\r\n\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    // Loop until Magnetometer, Gyroscope, and System reach level 3
    while( gyro < 3 || mag < 3 || accel < 3) {

        if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, &calib, 1, 10) != HAL_OK) {
            sprintf(msg, "\r\n[-] I2C Error: IMU Disconnected!\r\n");
            HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return 0;
        }

        sys   = (calib >> 6) & 0x03;
        gyro  = (calib >> 4) & 0x03;
        accel = (calib >> 2) & 0x03;
        mag   = (calib) & 0x03;

        sprintf(msg, "Status -> Sys:%d | Gyro:%d | Accel:%d | Mag:%d \r", sys, gyro, accel, mag);
        HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

        HAL_Delay(100);
    }

    sprintf(msg, "\r\n[+] IMU Calibration Complete!\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    // --- EXTRACT AND PRINT THE PROFILE ONCE CALIBRATION IS FINISHED ---
    uint8_t profile[22];
    IMU_GetCalibrationProfile(profile);

    sprintf(msg, "\r\n/* --- COPY THIS ARRAY INTO YOUR CODE --- */\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    sprintf(msg, "const uint8_t imu_calib_profile[22] = {\r\n    ");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    for(int i = 0; i < 22; i++) {
        sprintf(msg, "0x%02X", profile[i]);
        HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

        if (i < 21) {
            sprintf(msg, ", ");
            HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
        }
        if ((i + 1) % 8 == 0) { // Newline every 8 bytes for readability
            sprintf(msg, "\r\n    ");
            HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
        }
    }
    sprintf(msg, "\r\n};\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "/* -------------------------------------- */\r\n\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    return 1;
}

/**
 * @brief Reads Euler angles and Linear Acceleration safely
 */
uint8_t getOrientationIMU(IMU_Data_t *data) {
    uint8_t euler_buf[6];
    uint8_t accel_buf[6];

    if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_EUL_HEADING_LSB, I2C_MEMADD_SIZE_8BIT, euler_buf, 6, 10) != HAL_OK) return 0;

    int16_t h = (int16_t)((euler_buf[1] << 8) | euler_buf[0]);
    int16_t r = (int16_t)((euler_buf[3] << 8) | euler_buf[2]);
    int16_t p = (int16_t)((euler_buf[5] << 8) | euler_buf[4]);

    data->head  = (float)h / 16.0f;
    data->roll  = (float)r / 16.0f;
    data->pitch = (float)p / 16.0f;

    if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_LIA_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, accel_buf, 6, 10) != HAL_OK) return 0;

    int16_t ax = (int16_t)((accel_buf[1] << 8) | accel_buf[0]);
    int16_t ay = (int16_t)((accel_buf[3] << 8) | accel_buf[2]);
    int16_t az = (int16_t)((accel_buf[5] << 8) | accel_buf[4]);

    data->accelX = (float)ax / 100.0f;
    data->accelY = (float)ay / 100.0f;
    data->accelZ = (float)az / 100.0f;

    // --- READ GYROSCOPE ---
    uint8_t gyro_buffer[6];
    // FIXED: Changed hi2c to imu_i2c and added the missing semicolon
    if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_GYRO_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, gyro_buffer, 6, 10) != HAL_OK) return 0;

    int16_t gx = (int16_t)((gyro_buffer[1] << 8) | gyro_buffer[0]);
    int16_t gy = (int16_t)((gyro_buffer[3] << 8) | gyro_buffer[2]);
    int16_t gz = (int16_t)((gyro_buffer[5] << 8) | gyro_buffer[4]);

    // Convert to Degrees Per Second (dps). Default BNO055 scale is 16 LSB = 1 dps
    data->gyroX = (float)gx / 16.0f;
    data->gyroY = (float)gy / 16.0f;
    data->gyroZ = (float)gz / 16.0f;

    uint8_t calib = 0;
    HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, &calib, 1, 10);

    data->sys_calib   = (calib >> 6) & 0x03;
    data->gyro_calib  = (calib >> 4) & 0x03;
    data->accel_calib = (calib >> 2) & 0x03;
    data->mag_calib   = (calib) & 0x03;

    return 1;
}
