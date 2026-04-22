/*
 * sx1276.c
 *
 * Created on: Jan 26, 2026
 * Author: ted
 * Description: Driver implementation for SX1276 (TX Only, LoRa)
 */

#include "sx1276.h"
#include <string.h> // For generic memory operations if needed
#include <stdio.h>

// Helper macros for Chip Select
#define SX_NSS_LOW()   HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_RESET)
#define SX_NSS_HIGH()  HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_SET)

// -----------------------------------------------------------------------------
// LOW LEVEL SPI ACCESS
// -----------------------------------------------------------------------------

void SX1276_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t txData[2] = { reg | 0x80, value }; // Bit 7 set to 1 for Write access
    SX_NSS_LOW();
    HAL_SPI_Transmit(SX_SPI_HANDLE, txData, 2, 100);
    SX_NSS_HIGH();
}

uint8_t SX1276_ReadRegister(uint8_t reg) {
    uint8_t txData = reg & 0x7F; // Bit 7 cleared to 0 for Read access
    uint8_t rxData = 0;
    SX_NSS_LOW();
    HAL_SPI_Transmit(SX_SPI_HANDLE, &txData, 1, 100);
    HAL_SPI_Receive(SX_SPI_HANDLE, &rxData, 1, 100);
    SX_NSS_HIGH();
    return rxData;
}

// -----------------------------------------------------------------------------
// INITIALIZATION
// -----------------------------------------------------------------------------

void SX1276_Init(void) {
    // 1. Hardware Reset
    // Pulse the reset pin low for 10ms to ensure clean startup
    HAL_GPIO_WritePin(SX_RESET_PORT, SX_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SX_RESET_PORT, SX_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // 2. Set Sleep Mode
    // Transition to Sleep is required to switch between LoRa and FSK contexts
    SX1276_WriteRegister(REG_OP_MODE, MODE_SLEEP);
    HAL_Delay(1);

    // 3. Enable LoRa Mode
    // Set Bit 7 (LongRangeMode) while in Sleep
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);

    //  . Set the TX base adress at the bottom of the memory 0x00
    SX1276_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1276_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

    // 4. Enter Standby Mode
    // Registers should be configured in Standby
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    // Explicitly match Python's Sync Word (0x12)
    // SX1276_WriteRegister(REG_SYNC_WORD, 0x12);

    // 5. Configure Frequency: 869.53 MHz
    // Formula: Frf = (Freq * 2^19) / 32MHz
    // (869.53e6 * 524288) / 32e6 = 14246379.52 => 0xD961EB ou 0xD9664D
    SX1276_WriteRegister(REG_FRF_MSB, 0xD9);
    SX1276_WriteRegister(REG_FRF_MID, 0x61);
    SX1276_WriteRegister(REG_FRF_LSB, 0xEB);

    // 6. Configure Power Amplifier (PA)
    // Application uses a falling satellite -> Max robustness required.
    // PA_BOOST selected. Output Power set to 17dBm (0x0F).
    SX1276_WriteRegister(REG_PA_CONFIG, PA_BOOST | 0x0F);

    // 5.1. LNA
    // Low noise amplifier
    SX1276_WriteRegister(REG_LNA, 0x23); // Highest gain & Boost ON

    // 7. Over Current Protection (OCP)
    // Set to ~100mA to allow PA_BOOST to draw required current.
    SX1276_WriteRegister(REG_OCP, 0x2B);

    // 8. Modem Config 1: Bandwidth & Coding Rate
    // BW = 125 kHz: Good balance. Lower BW (e.g., 62.5k) is risky with Doppler/Motion.
    // CR = 4/8: Maximum Error Coding. Adds 100% overhead but allows recovery of corrupted bits.
    // Header: Explicit Mode (Bit 0 = 0).
    SX1276_WriteRegister(REG_MODEM_CONFIG_1, BW_125_KHZ | CR_4_8);

    // 9. Modem Config 2: Spreading Factor & CRC
    // SF = 10: High sensitivity (~-130dBm) for the 200m link.
    //          SF12 is slower and consumes more battery; SF10 is the sweet spot here.
    // CRC: Enabled (Bit 2 = 1). Vital for data integrity.
    SX1276_WriteRegister(REG_MODEM_CONFIG_2, SF_7 | 0x04);

    // 10. Modem Config 3: Optimization & LNA
    // LowDataRateOptimize: Not strictly required for SF10/125k (Symbol time < 16ms),
    // but we leave it at 0 (Auto/Off) unless you observe crystal drift issues.
    // AgcAutoOn (Bit 2) = 1: Auto Gain Control enabled (good practice).
    SX1276_WriteRegister(REG_MODEM_CONFIG_3, LOW_DATA_RATE_OPTIMIZE | AGC_AUTO_ON);

    // 11. Preamble Length
    // Set to 8 symbols (Standard).
    SX1276_WriteRegister(REG_PREAMBLE_MSB, 0x00);
    SX1276_WriteRegister(REG_PREAMBLE_LSB, 0x08);

    // DIO Mapping : DIO0 = TxDone
    SX1276_WriteRegister(REG_DIO_MAPPING_1, 0x40);
}

// -----------------------------------------------------------------------------
// TRANSMISSION
// -----------------------------------------------------------------------------

void SX1276_SendPacket(uint8_t *payload, uint8_t size) {
    // 1. Ensure Standby Mode
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    // 2. Set FIFO Pointers
    uint8_t txBase = SX1276_ReadRegister(REG_FIFO_TX_BASE_ADDR);
    SX1276_WriteRegister(REG_FIFO_ADDR_PTR, txBase);

    // 3. Write Payload Length
    SX1276_WriteRegister(REG_PAYLOAD_LENGTH, size);

    // 4. Write Payload to FIFO
    for (int i = 0; i < size; i++) {
        SX1276_WriteRegister(REG_FIFO, payload[i]);
    }

    // 5. Trigger Transmission
    SX1276_WriteRegister(REG_IRQ_FLAGS, 0xFF);
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // 6. SAFE RTOS WAIT (Timeout after 1 second to prevent MCU freeze!)
    uint32_t start_tx = HAL_GetTick();
    while (HAL_GetTick() - start_tx < 2000) {
        uint8_t irq = SX1276_ReadRegister(REG_IRQ_FLAGS);
        if (irq & IRQ_TX_DONE_MASK) {
            break; // TX Finished successfully!
        }
        osDelay(1); // CRITICAL: Yield CPU to the Sensor Task while we wait!
    }

    // 7. Clear IRQ Flags
    SX1276_WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

// -----------------------------------------------------------------------------
// TEMPERATURE SENSOR
// -----------------------------------------------------------------------------

int8_t SX1276_GetTemperature(void) {
    int8_t rawTemp = 0;
    int8_t temp = 0;

    // 1. Passage en mode SLEEP pour pouvoir basculer de LoRa à FSK
    SX1276_WriteRegister(REG_OP_MODE, 0x00); // RegOpMode: LongRangeMode=0 (FSK), Mode=Sleep
    HAL_Delay(1);

    // 2. Passage en mode STANDBY pour stabiliser le quartz
    SX1276_WriteRegister(REG_OP_MODE, 0x01); // RegOpMode: Mode=Stdby
    HAL_Delay(1);

    // 3. Passage en mode FSRx (Frequency Synthesis RX) - REQUIS pour le capteur
    // C'est l'étape qui manquait dans mon code précédent
    SX1276_WriteRegister(REG_OP_MODE, 0x04); // RegOpMode: Mode=FSRx
    HAL_Delay(1);

    // 4. Activer le moniteur de température (TempMonitorOff = 0)
    // Le registre REG_IMAGE_CAL est à l'adresse 0x3B. Le bit 0 est TempMonitorOff.
    // On lit l'existant pour ne pas écraser les autres réglages
    uint8_t imageCal = SX1276_ReadRegister(REG_IMAGE_CAL);
    SX1276_WriteRegister(REG_IMAGE_CAL, imageCal & 0xFE); // Bit 0 à 0 -> On allume le capteur

    // 5. Attendre au moins 140 microsecondes (stabilisation)
    HAL_Delay(1); // 1ms est largement suffisant pour couvrir les 140us

    // 6. Désactiver le moniteur de température (TempMonitorOff = 1)
    SX1276_WriteRegister(REG_IMAGE_CAL, imageCal | 0x01); // Bit 0 à 1 -> On éteint

    // 7. Retourner en mode SLEEP
    SX1276_WriteRegister(REG_OP_MODE, 0x00);

    // 8. Lire le résultat dans le registre RegTemp (0x3C)
    rawTemp = SX1276_ReadRegister(REG_TEMP);

    if ((rawTemp & 0x80)== 0x80)
    {
    	temp = 255 - rawTemp;
    }
    else
    {
    	temp = rawTemp;
    	temp *= -1;
    }

    // 9. Rebasculer en mode LoRa pour tes prochaines transmissions
    SX1276_WriteRegister(REG_OP_MODE, 0x80 | 0x00); // LoRa + Sleep
    SX1276_WriteRegister(REG_OP_MODE, 0x80 | 0x01); // LoRa + Standby

    // Conversion approximative (nécessite calibration réelle)
    // T [°C] = 1.2 * (rawTemp - 147) + 25 (formule type Semtech)
    return (int8_t)((25 - temp) + temp);
}

extern UART_HandleTypeDef huart1; // Bring in UART1 for the debug prints

void SX1276_SendCalibrationPacket(uint32_t calibrationDuration) {
    char msgBuffer[64];
    uint32_t timestamp = HAL_GetTick();

    // Format: CAL,duration,timestamp
    // Crucial: The \r\n MUST be here so Python knows the packet ended!
    sprintf(msgBuffer, "CAL,%lu,%lu\r\n", calibrationDuration, timestamp);

    // Transmit via LoRa
    SX1276_SendPacket((uint8_t*)msgBuffer, strlen(msgBuffer));

    // Echo to UART
    char debug_msg[128];
    sprintf(debug_msg, "[TX] Calibration Packet: CAL,%lu,%lu\r\n", calibrationDuration, timestamp);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
}
