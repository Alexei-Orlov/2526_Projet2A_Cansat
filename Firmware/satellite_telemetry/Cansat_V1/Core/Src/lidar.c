/*
 * lidar.c
 * Implementation for LightWare SF20/LW20 LiDAR
 */
#include "lidar.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "cansat_core.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// THE DMA buffer (this is the buffer that will do the ping pong
uint8_t lidar_dma_buf[DMA_BUFFER_SIZE];


static UART_HandleTypeDef *lidar_huart;
//static uint8_t rx_byte;

char lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
volatile uint16_t rx_index = 0;

// CHANGED TO 1: Defaults to silently parsing the stream during boot so it doesn't spam!
volatile uint8_t lidar_stream_mode = 1;

// Access the global queue defined in main.c
extern QueueHandle_t qSensorEvents;

void Lidar_Init(UART_HandleTypeDef *huart) {
    lidar_huart = huart;

    //DMA launch

    HAL_UART_Receive_DMA(lidar_huart, lidar_dma_buf, DMA_BUFFER_SIZE);
    /* Before the DMA update :
    // Clear any leftover junk in the UART registers and start the interrupt
    __HAL_UART_CLEAR_OREFLAG(lidar_huart);
    HAL_UART_Receive_IT(lidar_huart, &rx_byte, 1);*/
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
// Activates when the DMA buffer is half full
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        SensorEvent_t ev = EVENT_LIDAR_HALF_CPLT;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(qSensorEvents, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Activates when the DMA buffer is full
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        SensorEvent_t ev = EVENT_LIDAR_FULL_CPLT;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(qSensorEvents, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//===================================================================================================
// Processing of the DMA's incoming data
//===================================================================================================
void Process_Lidar_Buffer_Chunk(uint8_t* chunk_start, uint16_t chunk_length, uint32_t current_ms) {
    extern char lidar_cut_char[];
    extern uint8_t cut_char_len;
    extern LIDAR_Data_t latest_lidar;
    extern IMU_Data_t latest_imu;
    extern Barometer_Data_t latest_baro;
    extern GNSS_Data_t latest_gnss;
    extern QueueHandle_t qSDCard_LIDAR;
    extern CanSatState_t currentState;

    char local_line[64]; // Ligne complète assemblée
    uint8_t local_idx = 0;

    // On parcourt la moitié du buffer DMA, octet par octet
    for (uint16_t i = 0; i < chunk_length; i++) {
        char c = chunk_start[i];

        // On ignore les retours chariots orphelins
        if (c == '\r') continue;

        // Si on trouve une fin de ligne (un point complet est reçu !)
        if (c == '\n') {

            // 1. On assemble la ligne : Reliquat (s'il y en a) + Nouveaux caractères
            uint8_t total_len = 0;
            if (cut_char_len > 0) {
                memcpy(local_line, lidar_cut_char, cut_char_len);
                total_len += cut_char_len;
                cut_char_len = 0; // On vide le reliquat après l'avoir utilisé
            }
            if (local_idx > 0) {
                memcpy(local_line + total_len, chunk_start + (i - local_idx), local_idx);
                total_len += local_idx;
            }
            local_line[total_len] = '\0'; // On termine proprement la chaîne

            // On remet le compteur à zéro pour le point suivant
            local_idx = 0;

            // 2. EXTRACTION ULTRA-RAPIDE (Sans sscanf)
            // Format attendu du LightWare : "0.00\t12.34" (Angle et Distance)
            // On cherche le deuxième nombre (après l'espace ou la tabulation)
            char *distance_str = strpbrk(local_line, " \t");
            if (distance_str != NULL) {
                float dist_val = atof(distance_str);

                // Si la valeur est aberrante (ex: erreur de trame), on l'ignore
                if (dist_val >= 0.0f && dist_val < 150.0f) {
                    latest_lidar.distance = dist_val;
                    sprintf(latest_lidar.timestamp, "%lu", current_ms);

                    // --- ENVOI DIRECT À LA CARTE SD ---
                    if (currentState >= STATE_READY) {
                        LidarPacket_t lidar_pkt;
                        lidar_pkt.lidar     = latest_lidar;
                        lidar_pkt.roll      = latest_imu.roll;
                        lidar_pkt.pitch     = latest_imu.pitch;
                        lidar_pkt.yaw       = latest_imu.yaw;
                        lidar_pkt.height    = latest_baro.height;
                        lidar_pkt.latitude  = latest_gnss.latitude;
                        lidar_pkt.longitude = latest_gnss.longitude;
                        xQueueSend(qSDCard_LIDAR, &lidar_pkt, 0);
                    }
                }
            }
        }
        else {
            // C'est un chiffre normal, on incrémente le compteur
            local_idx++;
        }
    } // Fin de la boucle for

    // 3. GESTION DE LA PHRASE COUPÉE
    // Si la moitié du buffer s'est terminée au milieu d'un nombre (local_idx > 0)
    if (local_idx > 0) {
        // On sauvegarde ce fragment dans le reliquat pour la prochaine fois
        memcpy(lidar_cut_char, chunk_start + (chunk_length - local_idx), local_idx);
        cut_char_len = local_idx;
    }
}


/* Before the DMA update :
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
*/
/* The old debug it had an issue with the PC
 * *
 * @brief Pauses RTOS interrupts to create a direct bridge between PC and LiDAR.
 * Allows menu navigation using Spacebar and Arrow Keys.
 */
/*void Lidar_DirectDebug(UART_HandleTypeDef *huart_pc) {
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
}*/
void Lidar_UART_Error_Handler(UART_HandleTypeDef *huart) {
    // Nettoyage des erreurs matérielles
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    // Tuer la transaction DMA corrompue
    HAL_UART_AbortReceive(huart);

    // Relancer le LiDAR
    Lidar_Init(huart);
}

void Lidar_DirectDebug(UART_HandleTypeDef *huart_pc, UART_HandleTypeDef *huart_lidar) {

    // --- LA CORRECTION DU HARDFAULT ---
    // On assigne l'adresse matérielle au pointeur global sans armer les interruptions
    lidar_huart = huart_lidar;

    uint8_t pc_rx = 0;
    uint8_t lidar_rx = 0;
    char msg[] = "\r\n============================================\r\n"
                 "[*] LiDAR Debug Mode (SURVIVAL BARE-METAL)\r\n"
                 "[*] TAPE ESPACE POUR LE MENU, 'X' POUR QUITTER\r\n"
                 "============================================\r\n\n";

    HAL_UART_Transmit(huart_pc, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // --- LA BOUCLE DE SURVIE ---
    while (1) {

        // 1. GESTION INTELLIGENTE DES ERREURS
        if (__HAL_UART_GET_FLAG(huart_pc, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(huart_pc);
        }
        if (__HAL_UART_GET_FLAG(lidar_huart, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(lidar_huart);
        }

        // 2. LECTURE DU PC
        if (__HAL_UART_GET_FLAG(huart_pc, UART_FLAG_RXNE)) {

            HAL_UART_Receive(huart_pc, &pc_rx, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(huart_pc, &pc_rx, 1, HAL_MAX_DELAY); // Écho local

            // Condition de sortie
            if (pc_rx == 'X' || pc_rx == 'x') {
                break;
            }

            HAL_UART_Transmit(lidar_huart, &pc_rx, 1, HAL_MAX_DELAY);
        }

        // 3. LECTURE DU LIDAR
        if (__HAL_UART_GET_FLAG(lidar_huart, UART_FLAG_RXNE)) {

            HAL_UART_Receive(lidar_huart, &lidar_rx, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(huart_pc, &lidar_rx, 1, HAL_MAX_DELAY);
        }
    }

    char exit_msg[] = "\r\n\n[!] Sortie du mode Debug. Démarrage RTOS...\r\n";
    HAL_UART_Transmit(huart_pc, (uint8_t*)exit_msg, strlen(exit_msg), HAL_MAX_DELAY);
}
