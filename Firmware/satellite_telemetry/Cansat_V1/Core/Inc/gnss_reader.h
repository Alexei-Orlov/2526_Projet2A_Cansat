/*
 * gnss_reader.h
 * Description: NMEA sentence parser for GNSS/GPS modules
 */

#ifndef GNSS_READER_H
#define GNSS_READER_H

#include "main.h" // Adapted for STM32G4
#include <stdbool.h>

// Define USART according to CubeIDE configuration (USART1 for GPS/STLINK)
#define GNSS_HUART huart1

extern UART_HandleTypeDef GNSS_HUART;

// Maximum length of an NMEA sentence
#define NMEA_MAX_LEN 85

// Internal parsed data structure
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    uint8_t satellites;
    char time_utc[12]; // HHMMSS.ss
    bool fixed;        // true if valid signal (Status 'A')
} GNSS_Parsed_t;

// Global variable storing the latest parsed data
extern GNSS_Parsed_t parsed_gnss;

// --- API Functions ---
void GNSS_Init(void);
void GNSS_ConfigureM10_Boot(void);   /* stage 1: NMEA trimming only, engine stays at factory 1 Hz for acquisition */
void GNSS_ConfigureM10_Flight(void); /* stage 2: 10 Hz + airborne <2g, once the fix is stable */
void GNSS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GNSS_Process_Data(void);
void GNSS_UART_Error_Handler(UART_HandleTypeDef *huart);
void GNSS_CalibrateSatellite(void);

#endif // GNSS_READER_H
