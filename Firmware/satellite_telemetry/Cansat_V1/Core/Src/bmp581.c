/*
 * bmp581.c
 *
 *  Created on: Mar 8, 2025
 *      Author: mathi
 */

#include "bmp581.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "cansat_core.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t debug;

double bmptemp  = 0.0;
double bmppress = 0.0;
double bmpalt   = 0.0;
uint8_t odrcheck = 0;

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================*/

uint8_t bmp581_init_precise_normal(BMP_t *bmp581)
{
    int check = 0;

    /* OSR_P = x8: the INT/DRDY pin is not wired on this board, so hardware
       interrupts are not available — manual polling is used instead (20 Hz
       from TaskFSM). x8 gives ~155 Hz in continuous mode (datasheet Table 9),
       guaranteeing a fresh sample at every poll at a small noise cost
       (0.30 Pa RMS vs. 0.21 Pa at x16). */
    uint8_t OSR_mask = 0x58;  /* Pressure ON, OSR_P = x8, OSR_T = x1 */

    /* REG 0x31 DSP_IIR: bits[5:3] = set_iir_p, bits[2:0] = set_iir_t.
       Coefficient 15 on both channels (datasheet Table 10): at ~155 Hz ODR
       this gives a ~100 ms time constant (~1.5 m spatial lag at 15 m/s fall
       rate). Chosen to attenuate dynamic pressure artifacts from can rotation
       (Venturi effect at the vent) without over-smoothing the ~8 s descent
       profile. bits[5:3]=100 (0x20) for pressure, bits[2:0]=100 (0x04) for
       temperature. */
    uint8_t DSP_IIR_mask = 0x24;  /* IIR filter coeff 15 on pressure AND temperature */

    /* REG 0x30 DSP_CONFIG: bit5=shdw_sel_iir_p (already 1: pressure register
       returns filtered value), bit3=shdw_sel_iir_t (was 0: temperature register
       was returning raw value even with the filter active). 0x2B sets bit3=1
       so the read temperature is also the IIR-filtered value. */
    uint8_t DSP_conf_mask = 0x2B;  /* Read after IIR filter (P+T) + compensation ON */

    /* Continuous mode: sensor samples autonomously in a loop */
    uint8_t ODR_mask = 0x03;

    /* --- Write configuration registers (sensor must be in standby) --------- */
    if (HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_OSR_CONFIG, 1, &OSR_mask,      1, 10) != HAL_OK) check = 1;
    if (HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, 0x31 /* DSP_IIR */,  1, &DSP_IIR_mask, 1, 10) != HAL_OK) check = 1;
    if (HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_DSP_CONFIG,   1, &DSP_conf_mask,1, 10) != HAL_OK) check = 1;

    /* --- Wake the sensor only after all registers are set ------------------- */
    if (HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_ODR_CONFIG, 1, &ODR_mask, 1, 10) != HAL_OK) check = 1;

    /* --- Readback OSR_EFF to confirm the sensor is alive ------------------- */
    if (HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_OSR_EFF, 1, &odrcheck, 1, 10) != HAL_OK) check = 1;

    return check;
}

/* ===========================================================================
 * DATA READ
 * =========================================================================*/

uint8_t bmp581_read_precise_normal(BMP_t *bmp581)
{
    /* Manual polling (continuous mode, no interrupt): read the data registers
       directly without checking INT_STATUS first. */
    int check = 0;
    uint8_t recarray[6];
    int32_t intbuffertemp = 0;
    int32_t intbufferpres = 0;
    double tmoy = 0;

    /* Burst read: 6 bytes starting at TEMP_DATA_XLSB covers temp (3 bytes)
       then pressure (3 bytes) in one transaction.
       10 ms timeout: this transfer takes <1 ms at 100 kHz. The old 100 ms
       turned a wedged bus into 100 ms busy-wait spins at 20 Hz inside
       TaskSensors (highest priority), starving SD/LoRa/FSM of all CPU. */
    if (HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_TEMP_DATA_XLSB, 1, recarray, 6, 10) != HAL_OK) {
        check = 1;
    }

    if (check == 0) {
        intbuffertemp = (recarray[2] << 16) | (recarray[1] << 8) | recarray[0];
        intbufferpres = (recarray[5] << 16) | (recarray[4] << 8) | recarray[3];
        bmptemp  = (double)intbuffertemp / 65536.0;
        bmppress = (double)intbufferpres / 64.0;

        /* Two-pass altitude estimate: first pass uses standard atmosphere as
           mean temperature; second pass refines with the actual mean temperature
           between sea level and the estimated altitude. */
        bmpalt = ((8.314 * 293.15) / (9.80665 * 0.028964)) * log(101325.0 / bmppress);
        tmoy   = 293.15 + bmptemp + (0.0065 * bmpalt) / 2.0;
        bmpalt = ((8.314 * tmoy)   / (9.80665 * 0.028964)) * log(101325.0 / bmppress);
    }

    return check;
}

/* ===========================================================================
 * CALIBRATION
 * =========================================================================*/

/* Fixed ~8 s window rather than an adaptive wait for thermal stabilization:
   keeps the HMI's "please wait" screen (see startTaskHMI) bounded regardless
   of how long the die takes to settle. */
#define BARO_CALIB_SAMPLE_DELAY_MS  60
#define BARO_CALIB_DURATION_MS      8000
#define BARO_CALIB_TOTAL_SAMPLES    (BARO_CALIB_DURATION_MS / BARO_CALIB_SAMPLE_DELAY_MS)

/**
 * @brief  Measures ground-level pressure and temperature over a fixed window.
 *         Only the second half of samples is averaged to let the initial
 *         thermal transient settle before the reference is locked in.
 * @param  reference_pressure  Output: average pressure in Pa.
 * @param  reference_temp      Output: average temperature in °C.
 * @param  bmp581              Sensor handle.
 * @param  progress_cb         Optional callback receiving 0-100 (% elapsed). May be NULL.
 * @retval HAL_OK on success, HAL_ERROR if no valid sample was obtained.
 */
HAL_StatusTypeDef BMP581_CalibrateGroundPressure(double *reference_pressure, double *reference_temp,
                                                   BMP_t *bmp581, void (*progress_cb)(uint8_t percent))
{
    extern UART_HandleTypeDef huart1;
    extern osMutexId_t I2C1_MutexHandle;
    char msg[64];

    extern double bmppress;
    extern double bmptemp;

    double sum_pressure = 0.0;
    double sum_temp = 0.0;
    int valid_reads = 0;

    int discard_count = BARO_CALIB_TOTAL_SAMPLES / 2;

    for (int i = 0; i < BARO_CALIB_TOTAL_SAMPLES; i++) {
        /* Mutex taken per-sample, not held for the entire call, so the HMI task
           can still access the SSD1306 on I2C1 between reads. */
        if (osMutexAcquire(I2C1_MutexHandle, osWaitForever) == osOK) {
            uint8_t read_ok = (bmp581_read_precise_normal(bmp581) == 0);
            osMutexRelease(I2C1_MutexHandle);

            if (read_ok && i >= discard_count) {
                sum_pressure += bmppress;
                sum_temp     += bmptemp;
                valid_reads++;
            }
        }

        if (i % 20 == 0) {
            uint8_t percent = (uint8_t)((i * 100) / BARO_CALIB_TOTAL_SAMPLES);
            if (debug) {
                sprintf(msg, "[*] Calibrating... %d%%\r\n", percent);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            }
            if (progress_cb) progress_cb(percent);
        }
        osDelay(BARO_CALIB_SAMPLE_DELAY_MS);
    }

    if (valid_reads > 0) {
        *reference_pressure = sum_pressure / (double)valid_reads;
        *reference_temp     = sum_temp     / (double)valid_reads;
        if (progress_cb) progress_cb(100);
        return HAL_OK;
    }
    return HAL_ERROR;
}
