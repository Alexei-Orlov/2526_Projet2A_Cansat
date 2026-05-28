/*
 * hmi.h
 *
 *  Created on: 24 mai 2026
 *      Author: alexei
 */
#include "stm32g4xx_hal.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
extern I2C_HandleTypeDef hi2c1;
// États de page HMI
typedef enum {
    HMI_PAGE_OVERVIEW = 0,
    HMI_PAGE_MENU,
    HMI_PAGE_LORA,
    HMI_PAGE_SENSORS,
    HMI_PAGE_BATTERY
} HMI_Page_t;

// État interne HMI
typedef struct {
    HMI_Page_t current_page;
    uint8_t cursor_position;
    uint8_t lora_enabled;
} HMI_State_t;

// Données à afficher (mises à jour par TaskSensors/TaskFSM)
typedef struct {
    uint8_t baro_ready;
    uint8_t imu_ready;
    uint8_t gnss_ready;
    uint8_t lidar_ready;
    uint8_t gnss_satellites;
    float lidar_distance;
    float battery_voltage;
    uint8_t battery_percent;
} HMI_Display_Data_t;


void HMI_handle_button(HMI_State_t *state);
void HMI_exit_config_mode(void);
void HMI_toggle_lora(HMI_State_t *state);
void HMI_display_battery(HMI_State_t *state);
void HMI_display_sensors(HMI_State_t *state);
void HMI_display_lora(HMI_State_t *state);
void HMI_display_menu(HMI_State_t *state);
void HMI_display_overview(HMI_State_t *state);
uint8_t calculate_battery_percent(float voltage);
uint8_t HMI_safe_update_screen(void);

// Tableau conversion batterie
typedef struct {
    float voltage;
    uint8_t percent;
} BatteryLUT_t;


