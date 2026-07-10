/*
 * hmi.h
 *
 *  Created on: 24 mai 2026
 *      Author: alexei
 */

#ifndef INC_HMI_H_
#define INC_HMI_H_

#include "stm32g4xx_hal.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

extern I2C_HandleTypeDef hi2c1;

/* HMI page identifiers */
typedef enum {
    HMI_PAGE_OVERVIEW = 0,
    HMI_PAGE_MENU,
    HMI_PAGE_LORA,
    HMI_PAGE_SENSORS,
    HMI_PAGE_BATTERY,
    HMI_PAGE_RECALIB,    /* barometer recalibration */
    HMI_PAGE_FORMAT_SD   /* SD card format confirmation */
} HMI_Page_t;

/* Internal HMI state */
typedef struct {
    HMI_Page_t current_page;
    uint8_t    cursor_position;
    uint8_t    lora_enabled;
    uint8_t    format_sd_requested;  /* 0 = none, 1 = full format (f_mkfs),
                                        2 = quick erase (delete files only) */
    uint8_t    recalib_requested;    /* 1 = user requested recalibration */
} HMI_State_t;

/* Live display data updated by TaskSensors / TaskFSM */
typedef struct {
    uint8_t baro_ready;
    uint8_t imu_ready;
    uint8_t gnss_ready;
    uint8_t lidar_ready;
    uint8_t gnss_satellites;
    float   lidar_distance;
    float   battery_voltage;
    uint8_t battery_percent;
} HMI_Display_Data_t;

/* Voltage-to-percent lookup table entry */
typedef struct {
    float   voltage;
    uint8_t percent;
} BatteryLUT_t;

/* Display pages */
void HMI_display_overview(HMI_State_t *state);
void HMI_display_menu(HMI_State_t *state);
void HMI_display_lora(HMI_State_t *state);
void HMI_display_sensors(HMI_State_t *state);
void HMI_display_battery(HMI_State_t *state);
void HMI_display_recalib(HMI_State_t *state);
void HMI_display_recalib_progress(uint8_t percent);
void HMI_display_recalib_done(float pressure_pa, float temp_c);
void HMI_display_format_sd(HMI_State_t *state);
void HMI_display_flight_mode(void);

/* Actions */
void    HMI_handle_button(HMI_State_t *state);
void    HMI_exit_config_mode(void);
void    HMI_toggle_lora(HMI_State_t *state);
uint8_t HMI_safe_update_screen(void);
uint8_t calculate_battery_percent(float voltage);

#endif /* INC_HMI_H_ */
