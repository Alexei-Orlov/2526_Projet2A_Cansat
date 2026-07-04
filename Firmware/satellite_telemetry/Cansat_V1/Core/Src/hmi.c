/*
 * hmi.c
 *
 *  Created on: 24 mai 2026
 *      Author: alexei
 */

#include "hmi.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "cansat_core.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

HMI_Display_Data_t HMI_display_data = {0};
extern osMutexId_t I2C1_MutexHandle;  /* shared with the barometer on I2C1 */

static const BatteryLUT_t battery_lut[] = {
    {8.40f, 100},
    {7.94f,  80},
    {7.84f,  70},
    {7.74f,  60},
    {7.66f,  50},
    {7.58f,  40},
    {7.50f,  30},
    {7.40f,  20},
    {7.20f,  10},
    {6.60f,   5},
    {6.00f,   0}
};

#define BATTERY_LUT_SIZE (sizeof(battery_lut) / sizeof(BatteryLUT_t))

/* ===========================================================================
 * BATTERY LEVEL CONVERSION
 * =========================================================================*/

uint8_t calculate_battery_percent(float voltage)
{
    if (voltage >= battery_lut[0].voltage) return 100;
    if (voltage <= battery_lut[BATTERY_LUT_SIZE - 1].voltage) return 0;

    for (int i = 0; i < BATTERY_LUT_SIZE - 1; i++) {
        float v_high = battery_lut[i].voltage;
        float v_low  = battery_lut[i + 1].voltage;

        if (voltage <= v_high && voltage >= v_low) {
            float percent_high = (float)battery_lut[i].percent;
            float percent_low  = (float)battery_lut[i + 1].percent;

            float percent = percent_low +
                            (voltage - v_low) * (percent_high - percent_low) / (v_high - v_low);

            return (uint8_t)(percent + 0.5f);
        }
    }

    return 0;
}

uint8_t HMI_safe_update_screen(void)
{
    ssd1306_UpdateScreen();
    return 1;
}

/* ===========================================================================
 * DISPLAY PAGES
 * =========================================================================*/

void HMI_display_overview(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- CONFIG MODE --", Font_7x10, White);

    ssd1306_SetCursor(0, 14);
    if (state->lora_enabled) {
        ssd1306_WriteString("LoRa: ON  868MHz", Font_7x10, White);
    } else {
        if ((HAL_GetTick() / 500) % 2 == 0) {
            ssd1306_WriteString("LoRa: OFF 868MHz", Font_7x10, White);
        } else {
            ssd1306_WriteString("                ", Font_7x10, White);
        }
    }

    ssd1306_SetCursor(0, 26);
    ssd1306_WriteString(HMI_display_data.baro_ready ? "Baro: OK" : "Baro: CALIB...", Font_7x10, White);

    ssd1306_SetCursor(0, 36);
    ssd1306_WriteString(HMI_display_data.imu_ready  ? "IMU : OK" : "IMU : CALIB...", Font_7x10, White);

    ssd1306_SetCursor(0, 46);
    if (HMI_display_data.gnss_ready) {
        char buf[20];
        sprintf(buf, "GPS : %d sats", HMI_display_data.gnss_satellites);
        ssd1306_WriteString(buf, Font_7x10, White);
    } else {
        ssd1306_WriteString("GPS : WAIT...", Font_7x10, White);
    }

    /* Voltage split into int + decimal to avoid float in sprintf */
    ssd1306_SetCursor(0, 56);
    char bat_buf[20];
    int volt_int = (int)HMI_display_data.battery_voltage;
    int volt_dec = (int)((HMI_display_data.battery_voltage - volt_int) * 10);
    sprintf(bat_buf, "Bat:%d.%dV %d%%", volt_int, volt_dec, HMI_display_data.battery_percent);
    ssd1306_WriteString(bat_buf, Font_6x8, White);
}

void HMI_display_menu(HMI_State_t *state)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- MENU CONFIG --", Font_7x10, White);

    const char *menu_items[] = {
        "LoRa Setup",
        "Sensors Check",
        "Battery Info",
        "Recalibrate Baro",
        "Format SD",
        "Quit Config"
    };

    /* Scroll viewport: show 4 items at a time */
    uint8_t start_idx = 0;
    if (state->cursor_position >= 4) {
        start_idx = state->cursor_position - 3;
    }

    for (int i = 0; i < 4; i++) {
        uint8_t item_idx = start_idx + i;
        if (item_idx >= 6) break;

        ssd1306_SetCursor(0, 14 + i * 12);
        ssd1306_WriteString((state->cursor_position == item_idx) ? ">" : " ", Font_7x10, White);
        ssd1306_SetCursor(10, 14 + i * 12);
        ssd1306_WriteString((char*)menu_items[item_idx], Font_7x10, White);
    }

    ssd1306_SetCursor(0, 56);
    ssd1306_WriteString("Short=Nav Long=OK", Font_6x8, White);
}

void HMI_display_lora(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- LoRa Setup --", Font_7x10, White);

    ssd1306_SetCursor(0, 16);
    ssd1306_WriteString(state->lora_enabled ? "Etat : ON" : "Etat : OFF", Font_7x10, White);

    ssd1306_SetCursor(0, 28);
    ssd1306_WriteString("Freq : 868.53MHz", Font_7x10, White);

    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("SF10 BW125 CR4/8", Font_7x10, White);

    ssd1306_SetCursor(0, 52);
    ssd1306_WriteString((state->cursor_position == 0) ? "> ON/OFF Toggle" : "  ON/OFF Toggle", Font_6x8, White);

    ssd1306_SetCursor(0, 60);
    ssd1306_WriteString((state->cursor_position == 1) ? "> < Retour" : "  < Retour", Font_6x8, White);
}

void HMI_display_sensors(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Capteurs --", Font_7x10, White);

    ssd1306_SetCursor(0, 16);
    ssd1306_WriteString(HMI_display_data.baro_ready ? "BMP581  : OK" : "BMP581  : FAIL", Font_7x10, White);

    ssd1306_SetCursor(0, 26);
    ssd1306_WriteString(HMI_display_data.imu_ready  ? "ICM20948: OK" : "ICM20948: FAIL", Font_7x10, White);

    ssd1306_SetCursor(0, 36);
    char gnss_buf[20];
    if (HMI_display_data.gnss_ready) {
        sprintf(gnss_buf, "GNSS : %d sats", HMI_display_data.gnss_satellites);
    } else {
        sprintf(gnss_buf, "GNSS : WAIT");
    }
    ssd1306_WriteString(gnss_buf, Font_7x10, White);

    /* Distance split into int + decimal to avoid float in sprintf */
    ssd1306_SetCursor(0, 46);
    char lidar_buf[20];
    if (HMI_display_data.lidar_ready) {
        int lidar_int = (int)HMI_display_data.lidar_distance;
        int lidar_dec = (int)((HMI_display_data.lidar_distance - lidar_int) * 10);
        sprintf(lidar_buf, "LIDAR: %d.%dm", lidar_int, lidar_dec);
    } else {
        sprintf(lidar_buf, "LIDAR: N/A");
    }
    ssd1306_WriteString(lidar_buf, Font_7x10, White);

    ssd1306_SetCursor(0, 58);
    ssd1306_WriteString("< Retour", Font_6x8, White);
}

void HMI_display_battery(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Batterie --", Font_7x10, White);

    /* Voltage split into int + decimal to avoid float in sprintf */
    ssd1306_SetCursor(0, 18);
    char volt_buf[20];
    int volt_int = (int)HMI_display_data.battery_voltage;
    int volt_dec = (int)((HMI_display_data.battery_voltage - volt_int) * 100);
    sprintf(volt_buf, "Tension: %d.%02dV", volt_int, volt_dec);
    ssd1306_WriteString(volt_buf, Font_7x10, White);

    ssd1306_SetCursor(0, 30);
    char percent_buf[20];
    sprintf(percent_buf, "Niveau : %d%%", HMI_display_data.battery_percent);
    ssd1306_WriteString(percent_buf, Font_7x10, White);

    ssd1306_SetCursor(0, 42);
    ssd1306_WriteString("[", Font_7x10, White);

    int bar_length = (HMI_display_data.battery_percent * 10) / 100;
    char bar_char = (HMI_display_data.battery_percent < 20) ? '!'
                  : (HMI_display_data.battery_percent < 50) ? '-'
                  : '=';

    for (int i = 0; i < 10; i++) {
        ssd1306_SetCursor(7 + i * 7, 42);
        if (i < bar_length) {
            char buf[2] = {bar_char, '\0'};
            ssd1306_WriteString(buf, Font_7x10, White);
        } else {
            ssd1306_WriteString(" ", Font_7x10, White);
        }
    }
    ssd1306_SetCursor(77, 42);
    ssd1306_WriteString("]", Font_7x10, White);

    ssd1306_SetCursor(0, 56);
    ssd1306_WriteString("< Retour", Font_6x8, White);
}

/* ===========================================================================
 * HMI ACTIONS
 * =========================================================================*/

void HMI_toggle_lora(HMI_State_t *state)
{
    if (state->lora_enabled) {
        HAL_GPIO_WritePin(VPOWER_EN_GPIO_OUT_GPIO_Port, VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_RESET);
        state->lora_enabled = 0;

        ssd1306_Fill(Black);
        ssd1306_SetCursor(10, 24);
        ssd1306_WriteString("LoRa OFF", Font_11x18, White);
        HMI_safe_update_screen();
        osDelay(1000);
    } else {
        HAL_GPIO_WritePin(VPOWER_EN_GPIO_OUT_GPIO_Port, VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_SET);
        osDelay(100);

        state->lora_enabled = 1;

        ssd1306_Fill(Black);
        ssd1306_SetCursor(10, 24);
        ssd1306_WriteString("LoRa ON", Font_11x18, White);
        HMI_safe_update_screen();
        osDelay(1000);
    }
}

void HMI_exit_config_mode(void)
{
    extern volatile uint8_t configFlag;

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 10);
    ssd1306_WriteString("Quitting", Font_11x18, White);
    ssd1306_SetCursor(0, 32);
    ssd1306_WriteString("Config Mode", Font_11x18, White);
    HMI_safe_update_screen();
    osDelay(1500);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("DEBRANCHER", Font_11x18, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("l'ecran HMI", Font_11x18, White);
    ssd1306_SetCursor(0, 48);
    ssd1306_WriteString("avant vol !", Font_11x18, White);
    HMI_safe_update_screen();
    osDelay(3000);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 24);
    ssd1306_WriteString("READY!", Font_11x18, White);
    HMI_safe_update_screen();
    osDelay(1500);

    configFlag = 0;
}

/* ===========================================================================
 * BUTTON HANDLING
 * =========================================================================*/

void HMI_handle_button(HMI_State_t *state)
{
    uint32_t start_time = HAL_GetTick();

    /* Button uses a pull-down: pressed = GPIO_PIN_SET. Block until release. */
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
        osDelay(10);
    }

    uint32_t press_duration = HAL_GetTick() - start_time;

    /* Debounce: absorb spurious triggers from the falling-edge interrupt */
    if (press_duration < 50)
        return;

    /* --- short press: navigate (< 800 ms) --- */
    if (press_duration < 800) {
        switch (state->current_page) {
            case HMI_PAGE_OVERVIEW:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;
            case HMI_PAGE_MENU:
                state->cursor_position = (state->cursor_position + 1) % 6;
                break;
            case HMI_PAGE_LORA:
                state->cursor_position = (state->cursor_position + 1) % 2;
                break;
            case HMI_PAGE_FORMAT_SD:
                state->cursor_position = (state->cursor_position + 1) % 2;
                break;
            case HMI_PAGE_RECALIB:
                break;  /* no navigation on this page */
            case HMI_PAGE_SENSORS:
            case HMI_PAGE_BATTERY:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;
        }
    }
    /* --- long press: confirm action (>= 800 ms) --- */
    else {
        switch (state->current_page) {
            case HMI_PAGE_OVERVIEW:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;

            case HMI_PAGE_MENU:
                switch (state->cursor_position) {
                    case 0: state->current_page = HMI_PAGE_LORA;    state->cursor_position = 0; break;
                    case 1: state->current_page = HMI_PAGE_SENSORS; break;
                    case 2: state->current_page = HMI_PAGE_BATTERY; break;
                    case 3: state->current_page = HMI_PAGE_RECALIB; state->cursor_position = 0; break;
                    case 4: state->current_page = HMI_PAGE_FORMAT_SD; state->cursor_position = 1; break;  /* default to NO */
                    case 5: HMI_exit_config_mode(); break;
                }
                break;

            case HMI_PAGE_LORA:
                if (state->cursor_position == 0) {
                    HMI_toggle_lora(state);
                } else {
                    state->current_page = HMI_PAGE_MENU;
                    state->cursor_position = 0;
                }
                break;

            case HMI_PAGE_FORMAT_SD:
                if (state->cursor_position == 0) {
                    state->format_sd_requested = 1;
                } else {
                    state->current_page = HMI_PAGE_MENU;
                    state->cursor_position = 0;
                }
                break;

            case HMI_PAGE_RECALIB:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;

            case HMI_PAGE_SENSORS:
            case HMI_PAGE_BATTERY:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;
        }
    }
}

/* ===========================================================================
 * RECALIBRATION PAGE
 * =========================================================================*/

void HMI_display_recalib(HMI_State_t *state)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Baro Calib --", Font_7x10, White);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString("Calibrating...", Font_7x10, White);
    ssd1306_SetCursor(0, 32);
    ssd1306_WriteString("Do not move!", Font_7x10, White);
}

void HMI_display_recalib_progress(uint8_t percent)
{
    /* Redraw only the progress bar row to avoid full-screen flicker */
    ssd1306_FillRectangle(0, 44, 127, 56, Black);

    ssd1306_SetCursor(0, 44);
    ssd1306_WriteString("[", Font_7x10, White);

    int bar_length = (percent * 10) / 100;
    for (int i = 0; i < 10; i++) {
        ssd1306_SetCursor(7 + i * 7, 44);
        ssd1306_WriteString((i < bar_length) ? "=" : " ", Font_7x10, White);
    }
    ssd1306_SetCursor(77, 44);
    ssd1306_WriteString("]", Font_7x10, White);

    ssd1306_FillRectangle(90, 44, 127, 56, Black);
    char pct_buf[8];
    sprintf(pct_buf, " %d%%", percent);
    ssd1306_SetCursor(90, 44);
    ssd1306_WriteString(pct_buf, Font_7x10, White);
}

void HMI_display_recalib_done(float pressure_pa, float temp_c)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Baro Calib --", Font_7x10, White);

    ssd1306_SetCursor(0, 16);
    ssd1306_WriteString("Calibration OK!", Font_7x10, White);

    /* Pressure and temperature split into int + decimal to avoid float in sprintf */
    char buf[24];
    ssd1306_SetCursor(0, 30);
    int p_int = (int)pressure_pa;
    int p_dec = (int)((pressure_pa - p_int) * 10);
    sprintf(buf, "Ref: %d.%d Pa", p_int, p_dec);
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(0, 42);
    int t_int = (int)temp_c;
    int t_dec = (int)((temp_c - t_int) * 10);
    sprintf(buf, "Temp: %d.%d C", t_int, t_dec);
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_SetCursor(0, 56);
    ssd1306_WriteString("[BTN] -> Menu", Font_6x8, White);
}

/* ===========================================================================
 * FORMAT SD PAGE
 * =========================================================================*/

void HMI_display_format_sd(HMI_State_t *state)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Format SD? --", Font_7x10, White);

    ssd1306_SetCursor(0, 14);
    ssd1306_WriteString("WARNING: All data", Font_6x8, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("will be erased!", Font_6x8, White);

    ssd1306_SetCursor(0, 38);
    ssd1306_WriteString((state->cursor_position == 0) ? "> YES - Erase all" : "  YES - Erase all", Font_7x10, White);

    ssd1306_SetCursor(0, 52);
    ssd1306_WriteString((state->cursor_position == 1) ? "> NO  - Cancel" : "  NO  - Cancel", Font_7x10, White);
}
