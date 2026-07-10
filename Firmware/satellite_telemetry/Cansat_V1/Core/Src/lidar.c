/*
 * lidar.c
 * Implementation for LightWare SF20/LW20 LiDAR
 */

#include "lidar.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "cansat_core.h"
#include "gnss_reader.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t lidar_dma_buf[DMA_BUFFER_SIZE];  /* ping-pong DMA buffer */

static UART_HandleTypeDef *lidar_huart;

char lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;

/* Stream mode active by default so the boot sequence does not spam the log */
volatile uint8_t lidar_stream_mode = 1;

uint8_t lidar_is_connected = 0;

/* Tick of the last successfully parsed distance line (0 = none yet).
   Used by startTaskFSM's stream watchdog and by the HMI sensors page to
   distinguish "lines flowing right now" from "seen once since boot". */
volatile uint32_t lidar_last_data_ms = 0;

extern QueueHandle_t qSensorEvents;

/* ===========================================================================
 * INIT AND UART CALLBACKS
 * =========================================================================*/

void Lidar_Init(UART_HandleTypeDef *huart)
{
    lidar_huart = huart;
    HAL_UART_Receive_DMA(lidar_huart, lidar_dma_buf, DMA_BUFFER_SIZE);
}

void Lidar_RequestMenu(void)
{
    lidar_stream_mode = 0;
    uint8_t cmd[] = {' '};  /* spacebar opens the LiDAR configuration menu */
    HAL_UART_Transmit(lidar_huart, cmd, 1, 100);
}

void Lidar_RequestStream(void)
{
    lidar_stream_mode = 1;
    uint8_t cmd[] = {0x1B, 0x5B, 0x42};  /* VT100 Down Arrow triggers the data stream */
    HAL_UART_Transmit(lidar_huart, cmd, 3, 100);
}

/* Close any menu the LW20 might be sitting in, then start the stream.
   The LW20 keeps its internal state across an MCU-only reset (reflash /
   NRST), so unlike Lidar_RequestStream this cannot assume the device is
   at the top level: ESC ESC backs out of a possibly-open menu first.
   Harmless if the menu is already closed. */
void Lidar_ForceStream(void)
{
    if (lidar_huart == NULL) return;  /* not armed yet (boot sequence pending) */
    lidar_stream_mode = 1;
    uint8_t cmd[] = {0x1B, 0x1B, 0x5B, 0x42};  /* ESC, ESC, VT100 Down Arrow */
    HAL_UART_Transmit(lidar_huart, cmd, 4, 100);
}

/* DMA half-transfer: first 512 bytes ready */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        SensorEvent_t ev = EVENT_LIDAR_HALF_CPLT;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(qSensorEvents, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* DMA full-transfer: second 512 bytes ready.
   GNSS (USART1) uses single-byte interrupt reception, not DMA — route
   each completed byte to its own parser instead of the LIDAR path. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        SensorEvent_t ev = EVENT_LIDAR_FULL_CPLT;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(qSensorEvents, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else if (huart->Instance == USART1) {
        GNSS_UART_RxCpltCallback(huart);
    }
}

/* ===========================================================================
 * DMA BUFFER PROCESSING
 * =========================================================================*/

/* Parse one half of the DMA buffer (chunk_start, chunk_length bytes).
   Assembles complete LiDAR lines, extracts the distance field, and pushes
   a LidarPacket_t to qSDCard_LIDAR once the FSM is in STATE_READY. */
void Process_Lidar_Buffer_Chunk(uint8_t *chunk_start, uint16_t chunk_length, uint32_t current_ms)
{
    extern char lidar_cut_char[32];  /* sized to match main.c so sizeof() below works */
    extern uint8_t cut_char_len;
    extern LIDAR_Data_t latest_lidar;
    extern IMU_Data_t latest_imu;
    extern Barometer_Data_t latest_baro;
    extern GNSS_Data_t latest_gnss;
    extern QueueHandle_t qSDCard_LIDAR;
    extern CanSatState_t currentState;

    /* Atomic snapshot of IMU data: prevents a race condition with
       HAL_I2C_MemRxCpltCallback writing latest_imu from ISR context. */
    IMU_Data_t imu_snap;
    uint32_t   imu_snap_ms;
    taskENTER_CRITICAL();
        imu_snap    = latest_imu;
        imu_snap_ms = latest_imu.timestamp_ms;
    taskEXIT_CRITICAL();

    char    local_line[64];
    uint8_t local_idx = 0;

    for (uint16_t i = 0; i < chunk_length; i++) {
        char c = chunk_start[i];

        if (c == '\r') continue;

        if (c == '\n') {
            /* Assemble a full line from any leftover fragment + new characters.
               A line that cannot fit local_line is never a distance line
               (those are ~10 bytes) — it is LW20 menu output or framing
               garbage, so drop it entirely. Without this combined check the
               separate caps on the fragment (32) and local_idx (63) still
               allowed 32 + 63 + NUL = 96 bytes into this 64-byte stack
               buffer, corrupting the TaskSensors stack with symptoms that
               shifted on every recompile / power cycle. */
            uint8_t frag_len = cut_char_len;
            cut_char_len = 0;
            if ((uint16_t)frag_len + local_idx > sizeof(local_line) - 1) {
                local_idx = 0;
                continue;
            }

            uint8_t total_len = 0;
            if (frag_len > 0) {
                memcpy(local_line, lidar_cut_char, frag_len);
                total_len += frag_len;
            }
            if (local_idx > 0) {
                memcpy(local_line + total_len, chunk_start + (i - local_idx), local_idx);
                total_len += local_idx;
            }
            local_line[total_len] = '\0';
            local_idx = 0;

            /* LightWare format: "angle\tdistance" — extract the distance field */
            char *distance_str = strpbrk(local_line, " \t");
            if (distance_str != NULL) {
                float dist_val = atof(distance_str);

                /* Discard out-of-range values caused by framing errors */
                if (dist_val >= 0.0f && dist_val < 150.0f) {
                    latest_lidar.distance = dist_val;
                    sprintf(latest_lidar.timestamp, "%lu", current_ms);
                    lidar_is_connected = 1;
                    lidar_last_data_ms = current_ms;

                    if (currentState >= STATE_READY) {
                        LidarPacket_t lidar_pkt;
                        lidar_pkt.lidar      = latest_lidar;
                        lidar_pkt.roll       = imu_snap.roll;
                        lidar_pkt.pitch      = imu_snap.pitch;
                        lidar_pkt.yaw        = imu_snap.yaw;
                        lidar_pkt.quat_w     = imu_snap.quat_w;
                        lidar_pkt.quat_x     = imu_snap.quat_x;
                        lidar_pkt.quat_y     = imu_snap.quat_y;
                        lidar_pkt.quat_z     = imu_snap.quat_z;
                        lidar_pkt.accelX     = imu_snap.accelX;
                        lidar_pkt.accelY     = imu_snap.accelY;
                        lidar_pkt.accelZ     = imu_snap.accelZ;
                        lidar_pkt.imu_age_ms = current_ms - imu_snap_ms;  /* age of IMU sample at log time (ms) */
                        lidar_pkt.height     = latest_baro.height;
                        lidar_pkt.latitude   = latest_gnss.latitude;
                        lidar_pkt.longitude  = latest_gnss.longitude;
                        xQueueSend(qSDCard_LIDAR, &lidar_pkt, 0);
                    }
                }
            }
        } else {
            /* Bound-check before counting this byte: without a cap, a run of
               bytes this long without a '\n' (framing garbage from a baud
               mismatch) would overflow local_line[64] below via memcpy,
               corrupting the stack. Treat an overlong run as garbage and
               resync on the next '\n' instead. */
            if (local_idx < sizeof(local_line) - 1) {
                local_idx++;
            } else {
                local_idx = 0;
            }
        }
    }

    /* Save any fragment spanning the half-buffer boundary for the next call.
       Same overflow risk as above, into the 32-byte lidar_cut_char global —
       drop the fragment instead of overflowing if it doesn't fit. */
    if (local_idx > 0) {
        if (local_idx <= sizeof(lidar_cut_char)) {
            memcpy(lidar_cut_char, chunk_start + (chunk_length - local_idx), local_idx);
            cut_char_len = local_idx;
        } else {
            cut_char_len = 0;
        }
    }
}

/* ===========================================================================
 * ERROR HANDLING AND MAINTENANCE
 * =========================================================================*/

void Lidar_UART_Error_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    HAL_UART_AbortReceive(huart);
    Lidar_Init(huart);
}

/* Bare-metal passthrough loop for interactive LiDAR menu access.
   Assigns the hardware address to the global pointer without arming DMA
   interrupts, so the RTOS is not involved. Exit with 'X' or 'x'. */
void Lidar_DirectDebug(UART_HandleTypeDef *huart_pc, UART_HandleTypeDef *huart_lidar)
{
    lidar_huart = huart_lidar;

    uint8_t pc_rx    = 0;
    uint8_t lidar_rx = 0;
    char msg[] = "\r\n============================================\r\n"
                 "[*] LiDAR Debug Mode (bare-metal passthrough)\r\n"
                 "[*] PRESS SPACE FOR MENU, 'X' TO EXIT\r\n"
                 "============================================\r\n\n";

    HAL_UART_Transmit(huart_pc, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    while (1) {
        /* Clear overrun errors on both sides */
        if (__HAL_UART_GET_FLAG(huart_pc, UART_FLAG_ORE))
            __HAL_UART_CLEAR_OREFLAG(huart_pc);
        if (__HAL_UART_GET_FLAG(lidar_huart, UART_FLAG_ORE))
            __HAL_UART_CLEAR_OREFLAG(lidar_huart);

        /* PC -> LiDAR */
        if (__HAL_UART_GET_FLAG(huart_pc, UART_FLAG_RXNE)) {
            HAL_UART_Receive(huart_pc, &pc_rx, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(huart_pc, &pc_rx, 1, HAL_MAX_DELAY);  /* local echo */

            if (pc_rx == 'X' || pc_rx == 'x')
                break;

            HAL_UART_Transmit(lidar_huart, &pc_rx, 1, HAL_MAX_DELAY);
        }

        /* LiDAR -> PC */
        if (__HAL_UART_GET_FLAG(lidar_huart, UART_FLAG_RXNE)) {
            HAL_UART_Receive(lidar_huart, &lidar_rx, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(huart_pc, &lidar_rx, 1, HAL_MAX_DELAY);
        }
    }

    char exit_msg[] = "\r\n\n[!] Exiting debug mode. Starting RTOS...\r\n";
    HAL_UART_Transmit(huart_pc, (uint8_t*)exit_msg, strlen(exit_msg), HAL_MAX_DELAY);
}
