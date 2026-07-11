/*
 * sx1276.c
 *
 * Created on: Jan 26, 2026
 * Author: ted
 * Description: Driver implementation for SX1276 (TX Only, LoRa)
 */

#include "sx1276.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"

extern uint8_t debug;

#define SX_NSS_LOW()   HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_RESET)
#define SX_NSS_HIGH()  HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_SET)

/* ===========================================================================
 * LOW-LEVEL SPI ACCESS
 * =========================================================================*/

void SX1276_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = { reg | 0x80, value };  /* bit 7 = 1 for write */
    SX_NSS_LOW();
    HAL_SPI_Transmit(SX_SPI_HANDLE, txData, 2, 100);
    SX_NSS_HIGH();
}

uint8_t SX1276_ReadRegister(uint8_t reg)
{
    uint8_t txData = reg & 0x7F;  /* bit 7 = 0 for read */
    uint8_t rxData = 0;
    SX_NSS_LOW();
    HAL_SPI_Transmit(SX_SPI_HANDLE, &txData, 1, 100);
    HAL_SPI_Receive(SX_SPI_HANDLE, &rxData, 1, 100);
    SX_NSS_HIGH();
    return rxData;
}

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================*/

void SX1276_Init(void)
{
    /* Hardware reset: pulse low for 10 ms to ensure a clean startup */
    HAL_GPIO_WritePin(SX_RESET_PORT, SX_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SX_RESET_PORT, SX_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    SX_NSS_LOW();

    /* Transition to Sleep is required to switch from FSK to LoRa context */
    SX1276_WriteRegister(REG_OP_MODE, MODE_SLEEP);
    HAL_Delay(1);

    /* Set LongRangeMode bit (bit 7) while in Sleep */
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);

    SX1276_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1276_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

    /* Enter Standby before configuring registers */
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    /* Frequency: 869.53 MHz  (Frf = Freq * 2^19 / 32 MHz = 0xD961EB) */
    SX1276_WriteRegister(REG_FRF_MSB, 0xD9);
    SX1276_WriteRegister(REG_FRF_MID, 0x61);
    SX1276_WriteRegister(REG_FRF_LSB, 0xEB);

    /* PA_BOOST at 17 dBm: falling satellite requires maximum link robustness */
    SX1276_WriteRegister(REG_PA_CONFIG, PA_BOOST | 0x0F);

    /* LNA: highest gain + boost ON */
    SX1276_WriteRegister(REG_LNA, 0x23);

    /* OCP: ~100 mA to supply PA_BOOST */
    SX1276_WriteRegister(REG_OCP, 0x2B);

    /* Modem Config 1: BW = 125 kHz, CR = 4/8 (max error correction), explicit header */
    SX1276_WriteRegister(REG_MODEM_CONFIG_1, BW_125_KHZ | CR_4_8);

    /* Modem Config 2: SF10 (~-130 dBm sensitivity for the 200 m link), CRC enabled */
    SX1276_WriteRegister(REG_MODEM_CONFIG_2, SF_7 | 0x04);

    /* Modem Config 3: AGC auto ON; LowDataRateOptimize not required at SF10/125k */
    SX1276_WriteRegister(REG_MODEM_CONFIG_3, LOW_DATA_RATE_OPTIMIZE | AGC_AUTO_ON);

    /* Preamble: 8 symbols (standard) */
    SX1276_WriteRegister(REG_PREAMBLE_MSB, 0x00);
    SX1276_WriteRegister(REG_PREAMBLE_LSB, 0x08);

    /* DIO0 = TxDone interrupt */
    SX1276_WriteRegister(REG_DIO_MAPPING_1, 0x40);
}

/* ===========================================================================
 * TRANSMISSION
 * =========================================================================*/

void SX1276_SendPacket(uint8_t *payload, uint8_t size)
{
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    uint8_t txBase = SX1276_ReadRegister(REG_FIFO_TX_BASE_ADDR);
    SX1276_WriteRegister(REG_FIFO_ADDR_PTR, txBase);

    SX1276_WriteRegister(REG_PAYLOAD_LENGTH, size);

    for (int i = 0; i < size; i++) {
        SX1276_WriteRegister(REG_FIFO, payload[i]);
    }

    SX1276_WriteRegister(REG_IRQ_FLAGS, 0xFF);
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    /* Wait for TX_DONE flag; 2 s timeout guards against a frozen radio */
    uint32_t start_tx = HAL_GetTick();
    while (HAL_GetTick() - start_tx < 2000) {
        uint8_t irq = SX1276_ReadRegister(REG_IRQ_FLAGS);
        if (irq & IRQ_TX_DONE_MASK)
            break;
        osDelay(1);  /* yield to RTOS during wait */
    }

    SX1276_WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    SX1276_WriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/* ===========================================================================
 * TEMPERATURE SENSOR
 * =========================================================================*/

/* The SX1276 temperature monitor requires FSRx mode (not LoRa).
   This function temporarily switches the radio context, reads the sensor,
   then restores LoRa+Standby for subsequent transmissions.
   Sequence per datasheet section 3.5.7 (Rev. 7): Standby -> FSRx ->
   TempMonitorOff=0 -> wait >=140 us -> TempMonitorOff=1 -> Sleep/Standby ->
   read RegTemp. */

/* Uncalibrated absolute accuracy is +/-10 degC (datasheet 3.5.7). For a
   better reading, measure once against a known reference (e.g. the BMP581
   at ambient) and set this to (T_reference_degC - value returned with
   offset 0). */
#define SX1276_TEMP_CAL_OFFSET_C   0

int8_t SX1276_GetTemperature(void)
{
    int8_t rawTemp = 0;
    int8_t temp    = 0;

    /* Switch to sleep to exit the LoRa context */
    SX1276_WriteRegister(REG_OP_MODE, 0x00);  /* FSK mode + Sleep */
    HAL_Delay(1);

    /* Standby: let the crystal stabilize */
    SX1276_WriteRegister(REG_OP_MODE, 0x01);
    HAL_Delay(1);

    /* FSRx mode is required to activate the temperature monitor */
    SX1276_WriteRegister(REG_OP_MODE, 0x04);
    HAL_Delay(1);

    /* Enable temperature monitor (TempMonitorOff bit = 0);
       read-modify-write to preserve the other REG_IMAGE_CAL bits */
    uint8_t imageCal = SX1276_ReadRegister(REG_IMAGE_CAL);
    SX1276_WriteRegister(REG_IMAGE_CAL, imageCal & 0xFE);

    HAL_Delay(1);  /* wait >= 140 us for the sensor to settle */

    /* Disable temperature monitor */
    SX1276_WriteRegister(REG_IMAGE_CAL, imageCal | 0x01);

    SX1276_WriteRegister(REG_OP_MODE, 0x00);

    rawTemp = SX1276_ReadRegister(REG_TEMP);

    /* RegTemp slope is -1 degC/LSB (datasheet register table, 0x3C): the
       register decreases as the die heats up, so the sign must be flipped.
       Same decoding as the Semtech reference driver / RadioLib. */
    if ((rawTemp & 0x80) == 0x80) {
        temp = 255 - rawTemp;
    } else {
        temp = rawTemp;
        temp *= -1;
    }

    /* Restore LoRa mode */
    SX1276_WriteRegister(REG_OP_MODE, 0x80 | 0x00);  /* LoRa + Sleep */
    SX1276_WriteRegister(REG_OP_MODE, 0x80 | 0x01);  /* LoRa + Standby */

    /* The old return here was ((25 - temp) + temp), which is identically 25
       whatever the sensor reads. */
    return (int8_t)(temp + SX1276_TEMP_CAL_OFFSET_C);
}

/* ===========================================================================
 * CALIBRATION PACKET
 * =========================================================================*/

void SX1276_SendCalibrationPacket(uint32_t calibrationDuration)
{
    extern UART_HandleTypeDef huart1;

    char msgBuffer[64];
    uint32_t timestamp = HAL_GetTick();

    /* Format: CAL,duration,timestamp — \r\n marks the end of packet for the receiver */
    sprintf(msgBuffer, "CAL,%lu,%lu\r\n", calibrationDuration, timestamp);

    SX1276_SendPacket((uint8_t*)msgBuffer, strlen(msgBuffer));

    if (debug) {
        char debug_msg[128];
        sprintf(debug_msg, "[TX] Calibration Packet: CAL,%lu,%lu\r\n", calibrationDuration, timestamp);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    }
}
