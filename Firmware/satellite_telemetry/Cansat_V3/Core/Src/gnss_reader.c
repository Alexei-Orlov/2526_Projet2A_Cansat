/*
 * gnss_reader.c
 *
 *  Created on: Mar 19, 2026
 *      Author: juanp
 *  Description: NMEA sentence parser for GNSS/GPS modules
 */

#include "gnss_reader.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"

static char RxBuffer[NMEA_MAX_LEN];
static char SentenceBuffer[NMEA_MAX_LEN];
static uint8_t RxIndex = 0;

GNSS_Parsed_t parsed_gnss = {0};

/* ===========================================================================
 * INIT AND UART CALLBACKS
 * =========================================================================*/

void GNSS_Init(void)
{
    HAL_UART_Receive_IT(&GNSS_HUART, (uint8_t*)RxBuffer, 1);
}

void GNSS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != GNSS_HUART.Instance)
        return;

    if (RxBuffer[0] == '$')
        RxIndex = 0;

    SentenceBuffer[RxIndex++] = RxBuffer[0];

    if (RxIndex >= NMEA_MAX_LEN)
        RxIndex = 0;  /* Guard against malformed sentences without a newline */

    if (RxBuffer[0] == '\n') {
        SentenceBuffer[RxIndex] = '\0';
        RxIndex = 0;
        GNSS_Process_Data();
    }

    HAL_UART_Receive_IT(&GNSS_HUART, (uint8_t*)RxBuffer, 1);
}

/* Failsafe: restart single-byte reception if the hardware overrun flag fires. */
void GNSS_UART_Error_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    HAL_UART_Receive_IT(huart, (uint8_t*)RxBuffer, 1);
}

/* ===========================================================================
 * NMEA PARSING
 * =========================================================================*/

/* Extract the field at field_index from a comma-delimited NMEA sentence into
   result_buffer. The caller must ensure result_buffer has at least 20 bytes. */
static void GNSS_Get_Field(char *sentence, int field_index, char *result_buffer)
{
    result_buffer[0] = '\0';
    char *p = sentence;
    int comma_count = 0;

    while (p && comma_count < field_index) {
        p = strchr(p, ',');
        if (p) {
            p++;
            comma_count++;
        }
    }

    if (!p) return;

    int i = 0;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n' && i < 19)
        result_buffer[i++] = *p++;
    result_buffer[i] = '\0';
}

void GNSS_Process_Data(void)
{
    if (SentenceBuffer[0] != '$') return;

    /* RMC: position, speed, date/time (matches both $GPRMC and $GNRMC) */
    if (strstr(SentenceBuffer, "RMC")) {
        char time[20], status[20], lat_str[20], lat_dir[20], lon_str[20], lon_dir[20];

        GNSS_Get_Field(SentenceBuffer, 1, time);
        GNSS_Get_Field(SentenceBuffer, 2, status);

        if (strlen(time) > 0)
            strncpy(parsed_gnss.time_utc, time, 11);

        if (status[0] == 'A') {
            parsed_gnss.fixed = true;

            GNSS_Get_Field(SentenceBuffer, 3, lat_str);
            GNSS_Get_Field(SentenceBuffer, 4, lat_dir);
            GNSS_Get_Field(SentenceBuffer, 5, lon_str);
            GNSS_Get_Field(SentenceBuffer, 6, lon_dir);

            /* NMEA encodes lat/lon as DDDMM.MMMMM — convert to decimal degrees */
            if (strlen(lat_str) > 4) {
                float lat_val = atof(lat_str);
                int lat_deg   = (int)(lat_val / 100.0f);
                float lat_min = lat_val - (lat_deg * 100.0f);
                parsed_gnss.latitude = lat_deg + (lat_min / 60.0f);
                if (lat_dir[0] == 'S') parsed_gnss.latitude *= -1.0f;
            }

            if (strlen(lon_str) > 4) {
                float lon_val = atof(lon_str);
                int lon_deg   = (int)(lon_val / 100.0f);
                float lon_min = lon_val - (lon_deg * 100.0f);
                parsed_gnss.longitude = lon_deg + (lon_min / 60.0f);
                if (lon_dir[0] == 'W') parsed_gnss.longitude *= -1.0f;
            }
        } else {
            parsed_gnss.fixed = false;
        }
    }
    /* GGA: altitude and satellite count */
    else if (strstr(SentenceBuffer, "GGA")) {
        char sats[20], alt[20];

        GNSS_Get_Field(SentenceBuffer, 7, sats);
        if (strlen(sats) > 0) parsed_gnss.satellites = atoi(sats);

        GNSS_Get_Field(SentenceBuffer, 9, alt);
        if (strlen(alt) > 0) parsed_gnss.altitude = atof(alt);
    }
}

/* ===========================================================================
 * CALIBRATION
 * =========================================================================*/

/* Block until the GNSS reports an active fix with at least 4 satellites.
   Intended for pre-launch; blinks the status LED while searching. */
void GNSS_CalibrateSatellite(void)
{
    while (!parsed_gnss.fixed || parsed_gnss.satellites < 4) {
        HAL_GPIO_TogglePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin);
        osDelay(500);
    }

    HAL_GPIO_WritePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin, GPIO_PIN_SET);
}
