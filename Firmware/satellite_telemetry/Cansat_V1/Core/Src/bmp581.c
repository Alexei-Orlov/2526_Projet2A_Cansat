/*
 * bmp581.c
 *
 *  Created on: Mar 8, 2025
 *      Author: mathi
 */

#include "bmp581.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "cansat_core.h"

extern I2C_HandleTypeDef hi2c1;

double bmptemp=0.0;
double bmppress=0.0;
double bmpalt=0.0;
extern float temp;
uint8_t odrcheck=0;

//Ox18 donc 11000 pour ODR donc 5hz en mode normal avec oversampling a 128 pour la pression et 8 pour la temperature

uint8_t bmp581_init_precise_normal(BMP_t * bmp581){

//	uint8_t OSR_tmask = 0b01111111;
//	uint8_t ODR_tmask = 0b01100001;

	uint8_t OSR_tmask = 0b01111011;
	uint8_t ODR_tmask = 0b01101001;
	uint8_t DSP_conf_mask = 0b00101011;
	uint8_t DSP_conf_mask2 = 0b00010010;
	int check=0;

	if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_OSR_CONFIG, 1, &OSR_tmask, 1, 100)!=HAL_OK){
			check=1;
		}
	if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_ODR_CONFIG, 1, &ODR_tmask, 1, 100)!=HAL_OK){
					check=1;
				}
	if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_DSP_CONFIG, 1, &DSP_conf_mask, 1, 100)!=HAL_OK){
				check=1;
			}
	if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_DSP_CONFIG, 1, &DSP_conf_mask2, 1, 100)!=HAL_OK){
					check=1;
				}
	if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_OSR_EFF, 1, &odrcheck, 1, 100)!=HAL_OK){
					check=1;
				}

	return check;

}

uint8_t bmp581_read_precise_normal(BMP_t * bmp581){
		int check=0;
		uint8_t recarray[6];
		int32_t intbuffertemp=0;
		int32_t intbufferpres=0;

		double tmoy=0;
//		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_TEMP_DATA_XLSB, 1, &recarray[0], 1, 100)!=HAL_OK){
//			check=1;
//		}
//		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_TEMP_DATA_LSB, 1, &recarray[1], 1, 100)!=HAL_OK){
//			check=1;
//		}
//		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_TEMP_DATA_MSB, 1, &recarray[2], 1, 100)!=HAL_OK){
//			check=1;
//		}
//		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_PRESS_DATA_XLSB, 1, &recarray[3], 1, 100)!=HAL_OK){
//			check=1;
//		}
//		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_PRESS_DATA_LSB, 1, &recarray[4], 1, 100)!=HAL_OK){
//			check=1;
//		}
//		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_PRESS_DATA_MSB, 1, &recarray[5], 1, 100)!=HAL_OK){
//			check=1;
//		}
		if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_TEMP_DATA_XLSB, 1, recarray, 6, 100)!=HAL_OK){
					check=1;
				}


		if(check==0){

		intbuffertemp=(recarray[2]<<16)|(recarray[1]<<8)|(recarray[0]);
		intbufferpres=(recarray[5]<<16)|(recarray[4]<<8)|(recarray[3]);
		bmptemp=(double)intbuffertemp/65536.0;
		bmppress=(double) intbufferpres/64.0;

		//alt=(double)(288.15/0.0065)*(1-pow((double)(finalpress*1000.0)/101325.0, (double)(287.05*0.0065)/(9.80665)));
		bmpalt=(double) ((8.314*293.15)/(9.80665*0.028964))*log((double)101325.0/(bmppress));
		tmoy=(double) 293.15+bmptemp+(0.0065*bmpalt)/2;
		bmpalt=(double) ((8.314*tmoy)/(9.80665*0.028964))*log((double)101325.0/(bmppress));

		}

		return check;
}
// -----------------------------------------------------------------------------
// Calibration Function
// -----------------------------------------------------------------------------
/**
 * @brief  Takes multiple readings to establish the baseline ground pressure.
 * @param  reference_pressure: Pointer to float where the average pressure will be stored.
 * @retval HAL_OK if successful, HAL_ERROR if the sensor fails to read.
 */
HAL_StatusTypeDef BMP581_CalibrateGroundPressure(double *reference_pressure, double *reference_temp, BMP_t * bmp581) {
    double sum_pressure = 0.0;
    double sum_temp = 0.0;
    int valid_reads = 0;

    extern UART_HandleTypeDef huart1;
    char msg[64];

    extern double bmppress;
    extern double bmptemp;

    // Take 1000 readings to filter out noise
    for(int i = 0; i < SAMPLES_BAROMETER_CALIBRATION; i++) {
        if (bmp581_read_precise_normal(bmp581) == 0) {
            sum_pressure += bmppress;
            sum_temp += bmptemp;     // Also sum the temperature
            valid_reads++;
        }

        if (i % 100 == 0) {
            sprintf(msg, "[*] Calibrating... %d%%\r\n", (i/10));
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        }
        osDelay(60);
    }

    if (valid_reads > 0) {
        *reference_pressure = (sum_pressure / (double)valid_reads);
        *reference_temp = (sum_temp / (double)valid_reads);
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }
}
