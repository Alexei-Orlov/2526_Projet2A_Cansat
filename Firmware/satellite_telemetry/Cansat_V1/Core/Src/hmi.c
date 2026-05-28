/*
 * hmi.c
 *
 *  Created on: 24 mai 2026
 *      Author: alexei
 */


// ========== AJOUTS HMI ==========
#include "hmi.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "cansat_core.h"
HMI_Display_Data_t HMI_display_data = {0};

static const BatteryLUT_t battery_lut[] = {
    {8.40f, 100},
    {7.94f, 80},
    {7.84f, 70},
    {7.74f, 60},
    {7.66f, 50},
    {7.58f, 40},
    {7.50f, 30},
    {7.40f, 20},
    {7.20f, 10},
    {6.60f, 5},
    {6.00f, 0}
};

#define BATTERY_LUT_SIZE (sizeof(battery_lut) / sizeof(BatteryLUT_t))


// ========== CONVERSION BATTERIE ==========
uint8_t calculate_battery_percent(float voltage)  // ← uint8_t, pas char!
{
    if (voltage >= battery_lut[0].voltage) return 100;
    if (voltage <= battery_lut[BATTERY_LUT_SIZE - 1].voltage) return 0;

    for (int i = 0; i < BATTERY_LUT_SIZE - 1; i++) {
        float v_high = battery_lut[i].voltage;
        float v_low = battery_lut[i + 1].voltage;

        if (voltage <= v_high && voltage >= v_low) {
            float percent_high = (float)battery_lut[i].percent;
            float percent_low = (float)battery_lut[i + 1].percent;

            float percent = percent_low +
                           (voltage - v_low) * (percent_high - percent_low) / (v_high - v_low);

            return (uint8_t)(percent + 0.5f);
        }
    }

    return 0;
}

// ========== WRAPPER SÉCURISÉ I2C ==========
uint8_t hmi_safe_update_screen(void)  // ← uint8_t, pas char!
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 10) != HAL_OK) {
        return 0;
    }
    ssd1306_UpdateScreen();
    return 1;
}



// ========== WRAPPER SÉCURISÉ I2C ==========
uint8_t HMI_safe_update_screen(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 10) != HAL_OK) {
        return 0;
    }
    ssd1306_UpdateScreen();
    return 1;
}

// ========== AFFICHAGE PAGE OVERVIEW ==========
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
    if (HMI_display_data.baro_ready) {
        ssd1306_WriteString("Baro: OK", Font_7x10, White);
    } else {
        ssd1306_WriteString("Baro: CALIB...", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 36);
    if (HMI_display_data.imu_ready) {
        ssd1306_WriteString("IMU : OK", Font_7x10, White);
    } else {
        ssd1306_WriteString("IMU : CALIB...", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 46);
    if (HMI_display_data.gnss_ready) {
        char buf[20];
        sprintf(buf, "GPS : %d sats", HMI_display_data.gnss_satellites);
        ssd1306_WriteString(buf, Font_7x10, White);
    } else {
        ssd1306_WriteString("GPS : WAIT...", Font_7x10, White);
    }

    // ========== CORRECTION BATTERIE (SANS FLOAT) ==========
    ssd1306_SetCursor(0, 56);
    char bat_buf[20];
    int volt_int = (int)HMI_display_data.battery_voltage;
    int volt_dec = (int)((HMI_display_data.battery_voltage - volt_int) * 10);
    sprintf(bat_buf, "Bat:%d.%dV %d%%", volt_int, volt_dec, HMI_display_data.battery_percent);
    ssd1306_WriteString(bat_buf, Font_6x8, White);
}

// ========== AFFICHAGE MENU PRINCIPAL ==========
void HMI_display_menu(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- MENU CONFIG --", Font_7x10, White);

    const char* menu_items[] = {
        "LoRa Setup",
        "Sensors Check",
        "Battery Info",
        "Quit Config"
    };

    for (int i = 0; i < 4; i++) {
        ssd1306_SetCursor(0, 16 + i * 11);

        if (state->cursor_position == i) {
            ssd1306_WriteString(">", Font_7x10, White);
        } else {
            ssd1306_WriteString(" ", Font_7x10, White);
        }

        ssd1306_SetCursor(10, 16 + i * 11);
        ssd1306_WriteString((char*)menu_items[i], Font_7x10, White);
    }

    ssd1306_SetCursor(0, 56);
    ssd1306_WriteString("Court=Nav Long=OK", Font_6x8, White);
}

// ========== AFFICHAGE PAGE LORA ==========
void HMI_display_lora(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- LoRa Setup --", Font_7x10, White);

    ssd1306_SetCursor(0, 16);
    if (state->lora_enabled) {
        ssd1306_WriteString("Etat : ON", Font_7x10, White);
    } else {
        ssd1306_WriteString("Etat : OFF", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 28);
    ssd1306_WriteString("Freq : 868.53MHz", Font_7x10, White);

    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("SF10 BW125 CR4/8", Font_7x10, White);

    ssd1306_SetCursor(0, 52);
    if (state->cursor_position == 0) {
        ssd1306_WriteString("> ON/OFF Toggle", Font_6x8, White);
    } else {
        ssd1306_WriteString("  ON/OFF Toggle", Font_6x8, White);
    }

    ssd1306_SetCursor(0, 60);
    if (state->cursor_position == 1) {
        ssd1306_WriteString("> < Retour", Font_6x8, White);
    } else {
        ssd1306_WriteString("  < Retour", Font_6x8, White);
    }
}

// ========== AFFICHAGE PAGE SENSORS ==========
void HMI_display_sensors(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Capteurs --", Font_7x10, White);

    ssd1306_SetCursor(0, 16);
    ssd1306_WriteString(HMI_display_data.baro_ready ? "BMP581  : OK" : "BMP581  : FAIL", Font_7x10, White);

    ssd1306_SetCursor(0, 26);
    ssd1306_WriteString(HMI_display_data.imu_ready ? "ICM20948: OK" : "ICM20948: FAIL", Font_7x10, White);

    ssd1306_SetCursor(0, 36);
    char gnss_buf[20];
    if (HMI_display_data.gnss_ready) {
        sprintf(gnss_buf, "GNSS : %d sats", HMI_display_data.gnss_satellites);
    } else {
        sprintf(gnss_buf, "GNSS : WAIT");
    }
    ssd1306_WriteString(gnss_buf, Font_7x10, White);

    // ========== CORRECTION LIDAR (SANS FLOAT) ==========
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

// ========== AFFICHAGE PAGE BATTERY ==========
void HMI_display_battery(HMI_State_t *state)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Batterie --", Font_7x10, White);

    // ========== CORRECTION TENSION (SANS FLOAT) ==========
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
    char bar_char = '=';
    if (HMI_display_data.battery_percent < 20) {
        bar_char = '!';
    } else if (HMI_display_data.battery_percent < 50) {
        bar_char = '-';
    }

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
// ========== TOGGLE LORA ==========
void HMI_toggle_lora(HMI_State_t *state)
{
    if (state->lora_enabled) {
        HAL_GPIO_WritePin(VPOWER_EN_GPIO_OUT_GPIO_Port,
                         VPOWER_EN_GPIO_OUT_Pin,
                         GPIO_PIN_RESET);
        state->lora_enabled = 0;

        ssd1306_Fill(Black);
        ssd1306_SetCursor(10, 24);
        ssd1306_WriteString("LoRa OFF", Font_11x18, White);
        HMI_safe_update_screen();
        osDelay(1000);

    } else {
        HAL_GPIO_WritePin(VPOWER_EN_GPIO_OUT_GPIO_Port,
                         VPOWER_EN_GPIO_OUT_Pin,
                         GPIO_PIN_SET);
        osDelay(100);

        state->lora_enabled = 1;

        ssd1306_Fill(Black);
        ssd1306_SetCursor(10, 24);
        ssd1306_WriteString("LoRa ON", Font_11x18, White);
        HMI_safe_update_screen();
        osDelay(1000);
    }
}

// ========== SORTIE CONFIG ==========
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

// ========== GESTION BOUTON ==========
void HMI_handle_button(HMI_State_t *state)
{
    uint32_t press_duration = 0;
    uint32_t start_time = HAL_GetTick();

    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
        osDelay(10);
        press_duration = HAL_GetTick() - start_time;
        if (press_duration > 2000) break;
    }

    if (press_duration < 500) {

        switch (state->current_page) {
            case HMI_PAGE_OVERVIEW:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;

            case HMI_PAGE_MENU:
                state->cursor_position = (state->cursor_position + 1) % 4;
                break;

            case HMI_PAGE_LORA:
                state->cursor_position = (state->cursor_position + 1) % 2;
                break;

            case HMI_PAGE_SENSORS:
            case HMI_PAGE_BATTERY:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;
        }
    }
    else if (press_duration >= 1000) {

        switch (state->current_page) {
            case HMI_PAGE_OVERVIEW:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;

            case HMI_PAGE_MENU:
                switch (state->cursor_position) {
                    case 0:
                        state->current_page = HMI_PAGE_LORA;
                        state->cursor_position = 0;
                        break;
                    case 1:
                        state->current_page = HMI_PAGE_SENSORS;
                        break;
                    case 2:
                        state->current_page = HMI_PAGE_BATTERY;
                        break;
                    case 3:
                        HMI_exit_config_mode();
                        break;
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

            case HMI_PAGE_SENSORS:
            case HMI_PAGE_BATTERY:
                state->current_page = HMI_PAGE_MENU;
                state->cursor_position = 0;
                break;
        }
    }
}

