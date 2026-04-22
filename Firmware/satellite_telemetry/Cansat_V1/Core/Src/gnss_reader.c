/*
 * gnss_reader.c
 *
 *  Created on: Mar 19, 2026
 *      Author: juanp
 *  Description: NMEA sentence parser for GNSS/GPS modules
 */

#include "gnss_reader.h"
#include <string.h>
#include <stdlib.h> // For atof, atoi

// Buffers
static char RxBuffer[NMEA_MAX_LEN];
static char SentenceBuffer[NMEA_MAX_LEN];
static uint8_t RxIndex = 0;

GNSS_Parsed_t parsed_gnss = {0};

void GNSS_Init(void) {
    // Start listening for the first byte in the background
    HAL_UART_Receive_IT(&GNSS_HUART, (uint8_t*)RxBuffer, 1);
}

void GNSS_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == GNSS_HUART.Instance) {

        // Reset index if a new sentence starts
        if (RxBuffer[0] == '$') {
            RxIndex = 0;
        }

        SentenceBuffer[RxIndex++] = RxBuffer[0];

        // Prevent buffer overflow
        if (RxIndex >= NMEA_MAX_LEN) {
            RxIndex = 0;
        }

        // Process when newline is received
        if (RxBuffer[0] == '\n') {
            SentenceBuffer[RxIndex] = '\0';
            RxIndex = 0;
            GNSS_Process_Data();
        }

        // Re-arm the interrupt for the next byte
        HAL_UART_Receive_IT(&GNSS_HUART, (uint8_t*)RxBuffer, 1);
    }
}

// Helper function: Now requires you to pass a dedicated buffer!
static void GNSS_Get_Field(char* sentence, int field_index, char* result_buffer) {
    result_buffer[0] = '\0'; // Clear buffer
    char* p = sentence;
    int comma_count = 0;

    // Advance to the desired comma
    while (p && comma_count < field_index) {
        p = strchr(p, ',');
        if (p) {
            p++; // Skip the comma
            comma_count++;
        }
    }

    if (!p) return;

    // Copy until the next comma or end of string
    int i = 0;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n' && i < 19) {
        result_buffer[i++] = *p++;
    }
    result_buffer[i] = '\0';
}

void GNSS_Process_Data(void) {
    // Verify basic format (must start with $)
    if (SentenceBuffer[0] != '$') return;

    // Detect RMC sentence (either $GPRMC or $GNRMC)
    if (strstr(SentenceBuffer, "RMC")) {
        char time[20], status[20], lat_str[20], lat_dir[20], lon_str[20], lon_dir[20];

        GNSS_Get_Field(SentenceBuffer, 1, time);
        GNSS_Get_Field(SentenceBuffer, 2, status);

        if (strlen(time) > 0) {
            strncpy(parsed_gnss.time_utc, time, 11);
        }

        if (status[0] == 'A') {
            parsed_gnss.fixed = true;

            GNSS_Get_Field(SentenceBuffer, 3, lat_str);
            GNSS_Get_Field(SentenceBuffer, 4, lat_dir);
            GNSS_Get_Field(SentenceBuffer, 5, lon_str);
            GNSS_Get_Field(SentenceBuffer, 6, lon_dir);

            // Convert Latitude
            if (strlen(lat_str) > 4) {
                float lat_val = atof(lat_str);
                int lat_deg = (int)(lat_val / 100.0f);
                float lat_min = lat_val - (lat_deg * 100.0f);
                parsed_gnss.latitude = lat_deg + (lat_min / 60.0f);
                if (lat_dir[0] == 'S') parsed_gnss.latitude *= -1.0f;
            }

            // Convert Longitude
            if (strlen(lon_str) > 4) {
                float lon_val = atof(lon_str);
                int lon_deg = (int)(lon_val / 100.0f);
                float lon_min = lon_val - (lon_deg * 100.0f);
                parsed_gnss.longitude = lon_deg + (lon_min / 60.0f);
                if (lon_dir[0] == 'W') parsed_gnss.longitude *= -1.0f;
            }
        } else {
            parsed_gnss.fixed = false;
        }
    }
    // Detect GGA sentence (for Altitude and Satellites)
    else if (strstr(SentenceBuffer, "GGA")) {
        char sats[20], alt[20];

        GNSS_Get_Field(SentenceBuffer, 7, sats);
        if (strlen(sats) > 0) parsed_gnss.satellites = atoi(sats);

        GNSS_Get_Field(SentenceBuffer, 9, alt);
        if (strlen(alt) > 0) parsed_gnss.altitude = atof(alt);
    }
}
// Failsafe: Automatically restarts the UART if hardware overrun occurs
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == GNSS_HUART.Instance) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&GNSS_HUART, (uint8_t*)RxBuffer, 1);
    }
}


extern UART_HandleTypeDef huart1; // Bring in the PC UART for debug printing

void GNSS_CalibrateSatellite(void) {
    // Loop infinitely until we have an Active Fix AND at least 4 satellites
    while (!parsed_gnss.fixed || parsed_gnss.satellites < 4) {

        // Toggle LED rapidly to show we are alive and searching for satellites!
        HAL_GPIO_TogglePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin);

        // Yield to the RTOS for 500ms so background interrupts can update the parsed_gnss struct
        osDelay(500);
    }

    // Once we have a lock, turn the LED solid ON
    HAL_GPIO_WritePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin, GPIO_PIN_SET);
}
