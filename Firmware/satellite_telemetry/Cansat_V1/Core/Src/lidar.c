/*
 * lidar.c
 * Implementation for LightWare SF20/LW20 LiDAR
 */
#include "lidar.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "cansat_core.h"
#include <string.h>

static UART_HandleTypeDef *lidar_huart;
static uint8_t rx_byte;

char lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;

// CHANGED TO 1: Defaults to silently parsing the stream during boot so it doesn't spam!
volatile uint8_t lidar_stream_mode = 1;

// Access the global queue defined in main.c
extern QueueHandle_t qSensorEvents;

void Lidar_Init(UART_HandleTypeDef *huart) {
    lidar_huart = huart;

    // Clear any leftover junk in the UART registers and start the interrupt
    __HAL_UART_CLEAR_OREFLAG(lidar_huart);
    HAL_UART_Receive_IT(lidar_huart, &rx_byte, 1);
}

void Lidar_RequestMenu(void) {
    lidar_stream_mode = 0;
    uint8_t cmd[] = {' '}; // Spacebar opens the configuration menu
    HAL_UART_Transmit(lidar_huart, cmd, 1, 100);
}

void Lidar_RequestStream(void) {
    lidar_stream_mode = 1;
    uint8_t cmd[] = {0x1B, 0x5B, 0x42}; // Down Arrow triggers data stream
    HAL_UART_Transmit(lidar_huart, cmd, 3, 100);
}

// Automatically called by HAL_UART_RxCpltCallback in main.c
void Lidar_RxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == lidar_huart->Instance) {

        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                lidar_rx_buffer[rx_index] = '\0'; // Null-terminate the string
                rx_index = 0; // Reset for the next line

                // --- RTOS MAGIC: Send a ticket directly from the Interrupt! ---
                SensorEvent_t ev = EVENT_LIDAR_READY;
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(qSensorEvents, &ev, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        } else {
            // Protect against buffer overflows
            if (rx_index < LIDAR_RX_BUFFER_SIZE - 1) {
                lidar_rx_buffer[rx_index++] = rx_byte;
            }
        }

        // Re-arm the interrupt for the next byte
        HAL_UART_Receive_IT(lidar_huart, &rx_byte, 1);
    }
}

/**
 * @brief Pauses RTOS interrupts to create a direct bridge between PC and LiDAR.
 * Allows menu navigation using Spacebar and Arrow Keys.
 */
void Lidar_DirectDebug(UART_HandleTypeDef *huart_pc) {
    uint8_t pc_rx;
    uint8_t lidar_rx;
    char msg[] = "\r\n============================================\r\n"
                 "[*] LiDAR Direct Debug Mode\r\n"
                 "[*] PRESS SPACEBAR NOW to open the LiDAR menu!\r\n"
                 "[*] Use Arrow Keys to navigate.\r\n"
                 "[*] Type capital 'X' to exit and continue boot.\r\n"
                 "============================================\r\n\n";
    HAL_UART_Transmit(huart_pc, (uint8_t*)msg, strlen(msg), 100);

    // 1. Turn off the background interrupt
    HAL_UART_AbortReceive(lidar_huart);
    __HAL_UART_CLEAR_OREFLAG(lidar_huart);

    // Clear any distance strings that were mid-transmission out of our buffer
    while (HAL_UART_Receive(lidar_huart, &lidar_rx, 1, 0) == HAL_OK) {}

    // --- THE PASSTHROUGH LOOP ---
    while (1) {
        // Read from PC -> Send to LiDAR
        if (HAL_UART_Receive(huart_pc, &pc_rx, 1, 0) == HAL_OK) {
            if (pc_rx == 'X') {
                break; // Exit debug mode when user types capital X
            }
            HAL_UART_Transmit(lidar_huart, &pc_rx, 1, 5);
        }

        // Read from LiDAR -> Send to PC screen
        if (HAL_UART_Receive(lidar_huart, &lidar_rx, 1, 0) == HAL_OK) {
            HAL_UART_Transmit(huart_pc, &lidar_rx, 1, 5);
        }
    }

    char exit_msg[] = "\r\n\n[!] Exiting LiDAR Debug. Resuming boot...\r\n";
    HAL_UART_Transmit(huart_pc, (uint8_t*)exit_msg, strlen(exit_msg), 100);

    // If you left the menu open, send a spacebar to close it and resume the data stream
    uint8_t space = ' ';
    HAL_UART_Transmit(lidar_huart, &space, 1, 10);
    HAL_Delay(50);

    // Clean up any garbage bytes before returning to the RTOS
    __HAL_UART_CLEAR_OREFLAG(lidar_huart);
    while (HAL_UART_Receive(lidar_huart, &lidar_rx, 1, 0) == HAL_OK) {}

    // 3. Re-arm the background interrupt for flight mode!
    Lidar_Init(lidar_huart);
}
