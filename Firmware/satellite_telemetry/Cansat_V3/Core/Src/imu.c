/*
 * imu.c
 *
 * Created on: Jan 14, 2026
 * Author: juanp
 */

#include "imu.h"
#include <string.h>
#include <stdio.h>

extern uint8_t debug;

static I2C_HandleTypeDef *imu_i2c;

const uint8_t imu_calib_profile[22] = {
    0x01, 0x00, 0xFD, 0xFF, 0xFD, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00,
    0x00, 0x00, 0xE8, 0x03, 0x00, 0x00
};

static void BNO_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(imu_i2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

static void BNO_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10);
}

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================*/

uint8_t IMU_Init(I2C_HandleTypeDef *hi2c)
{
    imu_i2c = hi2c;
    uint8_t id = 0;

    /* BNO055 needs up to ~650ms after power-up (datasheet POR time) before it
       will respond correctly on I2C. This runs synchronously at the very
       start of boot, before the RTOS starts, so a cold power-cycle can reach
       this check before the sensor has finished its own boot. Retry for ~1s
       instead of a single, immediate, no-retry read. */
    uint8_t chip_found = 0;
    for (uint8_t attempt = 0; attempt < 10 && !chip_found; attempt++) {
        BNO_ReadRegs(BNO055_CHIP_ID_ADDR, &id, 1);
        if (id == 0xA0) {
            chip_found = 1;
        } else {
            HAL_Delay(100);
        }
    }
    if (!chip_found)
        return 0;  /* device not found */

    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_CONFIG);
    HAL_Delay(30);

    BNO_WriteReg(BNO055_SYS_TRIGGER_ADDR, 0x00);  /* use internal oscillator */
    HAL_Delay(50);

    IMU_SetCalibrationProfile(imu_calib_profile);
    HAL_Delay(30);

    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_NDOF);
    HAL_Delay(30);

    return 1;
}

void IMU_GetCalibrationProfile(uint8_t *profile)
{
    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_CONFIG);
    HAL_Delay(25);
    BNO_ReadRegs(BNO055_ACCEL_OFFSET_X_LSB, profile, 22);
    BNO_WriteReg(BNO055_OPR_MODE_ADDR, OPR_MODE_NDOF);
    HAL_Delay(25);
}

void IMU_SetCalibrationProfile(const uint8_t *profile)
{
    for (int i = 0; i < 22; i++) {
        BNO_WriteReg(BNO055_ACCEL_OFFSET_X_LSB + i, profile[i]);
        HAL_Delay(2);
    }
}

/* Blocking calibration routine — intended for one-time sessions only, not flight code.
   Prints progress to huart_debug and dumps the profile array when done. */
uint8_t IMU_calibration(UART_HandleTypeDef *huart_debug)
{
    uint8_t calib = 0;
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    char msg[128];

    sprintf(msg, "\r\n[*] IMU Calibration Started. Follow instructions:\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "1. Gyro:  Leave sensor perfectly still.\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "2. Mag:   Move sensor in a Figure-8 pattern.\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "3. Accel: Place resting on each of its 6 sides for 3 seconds.\r\n\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    while (gyro < 3 || mag < 3 || accel < 3) {

        if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_CALIB_STAT_ADDR,
                              I2C_MEMADD_SIZE_8BIT, &calib, 1, 10) != HAL_OK) {
            sprintf(msg, "\r\n[-] I2C Error: IMU Disconnected!\r\n");
            HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return 0;
        }

        sys   = (calib >> 6) & 0x03;
        gyro  = (calib >> 4) & 0x03;
        accel = (calib >> 2) & 0x03;
        mag   = (calib)      & 0x03;

        sprintf(msg, "Status -> Sys:%d | Gyro:%d | Accel:%d | Mag:%d \r", sys, gyro, accel, mag);
        HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

        HAL_Delay(100);
    }

    sprintf(msg, "\r\n[+] IMU Calibration Complete!\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    /* Extract and print the profile so it can be hardcoded into imu_calib_profile[] */
    uint8_t profile[22];
    IMU_GetCalibrationProfile(profile);

    sprintf(msg, "\r\n/* --- COPY THIS ARRAY INTO YOUR CODE --- */\r\n");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
    sprintf(msg, "const uint8_t imu_calib_profile[22] = {\r\n    ");
    HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);

    for (int i = 0; i < 22; i++) {
        sprintf(msg, "0x%02X", profile[i]);
        HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
        if (i < 21) {
            sprintf(msg, ", ");
            HAL_UART_Transmit(huart_debug, (uint8_t*)msg, strlen(msg), 100);
        }
        if ((i + 1) % 8 == 0) {
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

/* ===========================================================================
 * POLLING READ  (used only during calibration; flight code uses DMA path)
 * =========================================================================*/

uint8_t getOrientationIMU(IMU_Data_t *data)
{
    uint8_t euler_buf[6];
    uint8_t accel_buf[6];

    if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_EUL_HEADING_LSB,
                         I2C_MEMADD_SIZE_8BIT, euler_buf, 6, 10) != HAL_OK) return 0;

    int16_t h = (int16_t)((euler_buf[1] << 8) | euler_buf[0]);
    int16_t r = (int16_t)((euler_buf[3] << 8) | euler_buf[2]);
    int16_t p = (int16_t)((euler_buf[5] << 8) | euler_buf[4]);

    data->yaw   = (float)h / 16.0f;
    data->roll  = (float)r / 16.0f;
    data->pitch = (float)p / 16.0f;

    if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_LIA_DATA_X_LSB,
                         I2C_MEMADD_SIZE_8BIT, accel_buf, 6, 10) != HAL_OK) return 0;

    int16_t ax = (int16_t)((accel_buf[1] << 8) | accel_buf[0]);
    int16_t ay = (int16_t)((accel_buf[3] << 8) | accel_buf[2]);
    int16_t az = (int16_t)((accel_buf[5] << 8) | accel_buf[4]);

    data->accelX = (float)ax / 100.0f;
    data->accelY = (float)ay / 100.0f;
    data->accelZ = (float)az / 100.0f;

    uint8_t gyro_buffer[6];
    if (HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_GYRO_DATA_X_LSB,
                         I2C_MEMADD_SIZE_8BIT, gyro_buffer, 6, 10) != HAL_OK) return 0;

    int16_t gx = (int16_t)((gyro_buffer[1] << 8) | gyro_buffer[0]);
    int16_t gy = (int16_t)((gyro_buffer[3] << 8) | gyro_buffer[2]);
    int16_t gz = (int16_t)((gyro_buffer[5] << 8) | gyro_buffer[4]);

    /* BNO055 default gyroscope scale: 16 LSB = 1 dps */
    data->gyroX = (float)gx / 16.0f;
    data->gyroY = (float)gy / 16.0f;
    data->gyroZ = (float)gz / 16.0f;

    uint8_t calib_reg = 0;
    HAL_I2C_Mem_Read(imu_i2c, BNO055_I2C_ADDR, BNO055_CALIB_STAT_ADDR,
                     I2C_MEMADD_SIZE_8BIT, &calib_reg, 1, 10);

    data->sys_calib   = (calib_reg >> 6) & 0x03;
    data->gyro_calib  = (calib_reg >> 4) & 0x03;
    data->accel_calib = (calib_reg >> 2) & 0x03;
    data->mag_calib   = (calib_reg)      & 0x03;

    return 1;
}

/* ===========================================================================
 * DMA PATH  (flight mode: triggered at 100 Hz from TaskFSM)
 * =========================================================================*/

/* 34-byte burst: covers 0x14 (GyroX) through 0x35 (CalibStat) in one transfer */
static uint8_t imu_dma_rx_buf[34];

/* Trigger a non-blocking DMA read of the full sensor burst. */
uint8_t IMU_RequestData_DMA(void)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
        imu_i2c, BNO055_I2C_ADDR, BNO055_GYRO_DATA_X_LSB,
        I2C_MEMADD_SIZE_8BIT, imu_dma_rx_buf, 34);

    if (status != HAL_OK) {
        if (debug) {
            extern UART_HandleTypeDef huart1;
            char msg[128];
            sprintf(msg, "[-] DMA Fail! Stat:%d | State:0x%02X | Err:0x%02lX | DMA:%p\r\n",
                    status, imu_i2c->State, imu_i2c->ErrorCode, (void*)imu_i2c->hdmarx);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        }
        return 0;
    }
    return 1;
}

/* Unpack the DMA buffer into IMU_Data_t.
   Called from HAL_I2C_MemRxCpltCallback once the transfer completes. */
void IMU_ProcessData_DMA(IMU_Data_t *data)
{
    /* Gyroscope: bytes 0-5 -> registers 0x14-0x19 */
    int16_t gx = (int16_t)((imu_dma_rx_buf[1] << 8) | imu_dma_rx_buf[0]);
    int16_t gy = (int16_t)((imu_dma_rx_buf[3] << 8) | imu_dma_rx_buf[2]);
    int16_t gz = (int16_t)((imu_dma_rx_buf[5] << 8) | imu_dma_rx_buf[4]);

    data->gyroX = (float)gx / 16.0f;
    data->gyroY = (float)gy / 16.0f;
    data->gyroZ = (float)gz / 16.0f;

    /* Euler angles: bytes 6-11 -> registers 0x1A-0x1F */
    int16_t h = (int16_t)((imu_dma_rx_buf[7]  << 8) | imu_dma_rx_buf[6]);
    int16_t r = (int16_t)((imu_dma_rx_buf[9]  << 8) | imu_dma_rx_buf[8]);
    int16_t p = (int16_t)((imu_dma_rx_buf[11] << 8) | imu_dma_rx_buf[10]);

    data->yaw   = (float)h / 16.0f;
    data->roll  = (float)r / 16.0f;
    data->pitch = (float)p / 16.0f;

    /* Quaternion: bytes 12-19 -> registers 0x20-0x27.
       No gimbal lock; source of truth for 3D reconstruction (pitch reaches +-139 deg in flight). */
    int16_t qw = (int16_t)((imu_dma_rx_buf[13] << 8) | imu_dma_rx_buf[12]);
    int16_t qx = (int16_t)((imu_dma_rx_buf[15] << 8) | imu_dma_rx_buf[14]);
    int16_t qy = (int16_t)((imu_dma_rx_buf[17] << 8) | imu_dma_rx_buf[16]);
    int16_t qz = (int16_t)((imu_dma_rx_buf[19] << 8) | imu_dma_rx_buf[18]);

    /* BNO055 quaternion scale: 1 unit = 2^14 LSB (datasheet Table 3-31) */
    data->quat_w = (float)qw / 16384.0f;
    data->quat_x = (float)qx / 16384.0f;
    data->quat_y = (float)qy / 16384.0f;
    data->quat_z = (float)qz / 16384.0f;

    /* Linear acceleration: bytes 20-25 -> registers 0x28-0x2D */
    int16_t ax = (int16_t)((imu_dma_rx_buf[21] << 8) | imu_dma_rx_buf[20]);
    int16_t ay = (int16_t)((imu_dma_rx_buf[23] << 8) | imu_dma_rx_buf[22]);
    int16_t az = (int16_t)((imu_dma_rx_buf[25] << 8) | imu_dma_rx_buf[24]);

    data->accelX = (float)ax / 100.0f;
    data->accelY = (float)ay / 100.0f;
    data->accelZ = (float)az / 100.0f;

    /* Bytes 26-32 (gravity vector + temperature) are not used */

    /* Calibration status: byte 33 -> register 0x35 */
    uint8_t calib = imu_dma_rx_buf[33];
    data->sys_calib   = (calib >> 6) & 0x03;
    data->gyro_calib  = (calib >> 4) & 0x03;
    data->accel_calib = (calib >> 2) & 0x03;
    data->mag_calib   = (calib)      & 0x03;
}
