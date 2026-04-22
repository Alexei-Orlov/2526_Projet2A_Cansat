/*
 * cansat_core.h
 *
 * Created on: Mar 10, 2026
 * Author: juanp
 */

#ifndef CANSAT_CORE_H
#define CANSAT_CORE_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "imu.h"

// ====================================================================
// --- MISSION LOGIC THRESHOLDS
// ====================================================================
#define THRESH_LIDAR_IN_BOX_M      		0.3f  	// Max distance to be considered "inside the deployment box"
#define THRESH_LIDAR_DEPLOYED_M    		1.0f  	// Min distance to confirm deployment out of the box
#define THRESH_ALTITUDE_LAUNCH_M   		0.3f  	// Min altitude to trigger ASCENSION state
#define THRESH_ALTITUDE_LANDING_M  		5.0f  	// Max altitude to trigger RECOVERY state (buzzer on)
#define SAMPLES_BAROMETER_CALIBRATION  	100  	// Samples in order to determine the calibration state of the barometer
#define TIME_BETWEEN_PACKET_LORA_mS 	100 	// Time between each packet sent through the lora

// ====================================================================

// --- SENSOR DATA STRUCTURES ---
typedef struct {
    char timestamp[16];
    float latitude;
    float longitude;
    float altitude;
    uint8_t satellites;
} GNSS_Data_t;

typedef struct {
    char timestamp[16];
    float height;
    float temperature;
} Barometer_Data_t;

typedef struct {
    char timestamp[16];
    float distance;
} LIDAR_Data_t;

// Composite struct for SD and LoRa Queues
typedef struct {
    GNSS_Data_t gnss;
    Barometer_Data_t baro;
    IMU_Data_t imu;
    LIDAR_Data_t lidar;
} TelemetryPacket_t;

// --- SENSOR EVENT ENUM ---
typedef enum {
    EVENT_GNSS_READY,
    EVENT_BARO_READY,
    EVENT_IMU_READY,
    EVENT_LIDAR_READY
} SensorEvent_t;

// --- FSM STATES ---
typedef enum {
    STATE_STANDBY,
    STATE_CONFIG,
    STATE_READY,
    STATE_ASCENSION,
    STATE_DROP,
    STATE_RECOVERY,
    STATE_OFF
} CanSatState_t;

// --- RTOS HANDLES ---
extern QueueHandle_t qSensorEvents;
extern QueueHandle_t qSDCard;
extern QueueHandle_t qLoRa;

extern CanSatState_t currentState;
extern float current_height; // Global for FSM transitions

#endif
