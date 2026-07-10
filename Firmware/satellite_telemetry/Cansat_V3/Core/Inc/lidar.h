/*
 * lidar.h
 * Driver for LightWare SF20/LW20 LiDAR (DMA ping-pong, USART3)
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "stm32g4xx_hal.h"

#define LIDAR_RX_BUFFER_SIZE 128
#define DMA_BUFFER_SIZE      200

/* Shared globals accessed by Process_Lidar_Buffer_Chunk and startTaskSensors */
extern char             lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
extern volatile uint8_t lidar_stream_mode;
/* Set once a first valid distance line has been parsed; used by the HMI
   sensors page to show LiDAR connectivity. */
extern uint8_t lidar_is_connected;
/* Tick of the last successfully parsed distance line (0 = none yet).
   Lets callers detect a live stream vs. a stale one. */
extern volatile uint32_t lidar_last_data_ms;

void Lidar_Init(UART_HandleTypeDef *huart);
void Lidar_RequestMenu(void);
void Lidar_RequestStream(void);
void Lidar_ForceStream(void);
void Lidar_DirectDebug(UART_HandleTypeDef *huart_pc, UART_HandleTypeDef *huart_lidar);
void Lidar_UART_Error_Handler(UART_HandleTypeDef *huart);
void Process_Lidar_Buffer_Chunk(uint8_t *chunk_start, uint16_t chunk_length, uint32_t current_ms);

#endif /* INC_LIDAR_H_ */
