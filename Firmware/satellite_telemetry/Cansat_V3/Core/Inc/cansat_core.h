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
#define THRESH_LIDAR_DEPLOYED_M    		1.0f  	// Min distance to confirm deployment out of the box
#define THRESH_ALTITUDE_DEPLOY_MIN_M	30.0f 	// Min altitude for the deployment (DROP) detection — release is nominally at 120 m
#define THRESH_ALTITUDE_ASCENSION_M		10.0f 	// Min altitude to trigger ASCENSION state
#define THRESH_ASCENSION_HOLD_MS   		2000  	// Ascension condition must hold this long before READY -> ASCENSION
#define THRESH_DROP_HOLD_MS        		1000  	// Deployment condition must hold this long before ASCENSION -> DROP
#define THRESH_ALTITUDE_LANDING_M  		5.0f  	// Max altitude to trigger RECOVERY state (buzzer on)
#define THRESH_GYRO_STILL_DPS      		30.0f 	// Max |gyro| on every axis to consider the CanSat motionless (landing confirmation)
#define THRESH_LANDING_HOLD_MS     		3000  	// Landing condition must hold this long before DROP -> RECOVERY
/* Outlier tolerance per hold window: samples violating the condition are
   counted instead of resetting the window; one above the cap aborts it.
   The FSM samples at 20 Hz, so the windows above hold 40 / 20 / 60 samples
   and the caps below allow ~10% of noisy samples (baro glitch, gyro gust). */
#define THRESH_ASCENSION_MAX_OUTLIERS	4   	// of 40 samples (20 Hz x 2 s)
#define THRESH_DROP_MAX_OUTLIERS   		2   	// of 20 samples (20 Hz x 1 s)
#define THRESH_LANDING_MAX_OUTLIERS		6   	// of 60 samples (20 Hz x 3 s)
#define LOG_CLOSE_AFTER_RECOVERY_MS		500000	// SD log files are closed this long after entering RECOVERY (card then safe to pull)
#define TIME_BETWEEN_PACKET_LORA_mS 	100 	// Time between each packet sent through the lora
#define HMI_MENU_ITEM_COUNT 6

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

typedef struct {
    float voltage;
} Battery_Data_t;

// Composite struct for SD and LoRa Queues
typedef struct {
    GNSS_Data_t gnss;
    Barometer_Data_t baro;
    IMU_Data_t imu;
    LIDAR_Data_t lidar;
    Battery_Data_t bat;
} TelemetryPacket_t;

typedef struct {
    GNSS_Data_t gnss;
    Barometer_Data_t baro;
    IMU_Data_t imu;
    LIDAR_Data_t lidar;
} FastPacket_t;

// Lightweight packet for LIDAR queue (50Hz) - saves RAM vs FastPacket_t
typedef struct {
    LIDAR_Data_t lidar;
    float roll;
    float pitch;
    float yaw;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float accelX;
    float accelY;
    float accelZ;
    float height;
    float latitude;
    float longitude;
    float gnss_altitude; // MSL altitude from the SAM-M10Q (10 Hz GGA), for georeferencing the point cloud
    uint16_t imu_age_ms; // diagnostic: age of the IMU sample when the LIDAR packet was built (ms)
} LidarPacket_t;

// --- SENSOR EVENT ENUM ---
typedef enum {
    EVENT_GNSS_READY,
    EVENT_BARO_READY,
    EVENT_IMU_READY,
    //EVENT_LIDAR_READY, Before the DMA update
	EVENT_LIDAR_HALF_CPLT,
	EVENT_LIDAR_FULL_CPLT,
    EVENT_BATTERY_READY
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
extern QueueHandle_t qSDCard_LIDAR;
extern QueueHandle_t qLoRa;
extern QueueHandle_t qHMI_Events;

extern CanSatState_t currentState;
extern float current_height; // Global for FSM transitions

#endif
