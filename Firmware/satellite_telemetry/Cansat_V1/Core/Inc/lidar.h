/*
 * lidar.h
 * Implementation for LightWare SF20/LW20 LiDAR
 */
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "stm32g4xx_hal.h"

#define LIDAR_RX_BUFFER_SIZE 128
#define DMA_BUFFER_SIZE 200
// Shared variables for the RTOS Sensor Task
extern char lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
extern volatile uint8_t lidar_stream_mode;

void Lidar_Init(UART_HandleTypeDef *huart);
void Lidar_RequestMenu(void);
void Lidar_RequestStream(void);
void Lidar_RxCallback(UART_HandleTypeDef *huart);

// NEW: Direct Console Communication
// Remplace l'ancienne définition par celle-ci :
void Lidar_DirectDebug(UART_HandleTypeDef *huart_pc, UART_HandleTypeDef *huart_lidar);

void Lidar_UART_Error_Handler(UART_HandleTypeDef *huart);

// Prototype du parseur DMA
void Process_Lidar_Buffer_Chunk(uint8_t* chunk_start, uint16_t chunk_length, uint32_t current_ms);
#endif /* INC_LIDAR_H_ */
