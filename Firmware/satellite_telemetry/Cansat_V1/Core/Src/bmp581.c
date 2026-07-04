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

uint8_t bmp581_init_precise_normal(BMP_t * bmp581){
    int check = 0;

    // 1. Masques de configuration
    // OSR_P = x8 (au lieu de x16) : la broche INT/DRDY ne réagit pas matériellement
    // sur cette carte, donc plus d'interruption — on revient au polling manuel
    // (cf. TaskFSM, appel à 100Hz comme l'IMU). x8 donne ~155Hz en mode continu
    // (table 9 datasheet), ce qui garantit un échantillon frais à chaque poll
    // 100Hz, au prix d'un bruit légèrement supérieur (0.30 Pa RMS vs 0.21 Pa en x16).
    uint8_t OSR_mask      = 0x58; // Pression ON, OSR_P = x8, OSR_T = x1
    // Reg 0x31 DSP_IIR : set_iir_p occupe les bits[5:3], set_iir_t les bits[2:0].
    // Valeur 0x4 = coefficient 15 sur chaque voie (table 10 datasheet) : avec
    // l'ODR interne ~155Hz (OSR_P=x8), constante de temps ≈ 100ms, soit ≈1.5m
    // de décalage spatial à 15m/s (vitesse de chute visée). Compromis choisi
    // pour couper un éventuel parasite de pression dynamique lié à la
    // rotation de la canette (effet Venturi au niveau de l'évent) sans trop
    // lisser le profil réel de descente sur les ~8s de vol. bits[5:3]=100
    // (0x20) pour set_iir_p, bits[2:0]=100 (0x04) pour set_iir_t.
    uint8_t DSP_IIR_mask  = 0x24; // Filtre IIR (coeff 15) sur pression ET température
    // Reg 0x30 DSP_CONFIG : bit5=shdw_sel_iir_p (déjà à 1 : registre pression
    // = valeur filtrée), bit3=shdw_sel_iir_t (était à 0 : le registre
    // température restait brut même avec le filtre activé). 0x2B ajoute
    // bit3=1 pour que la température lue soit elle aussi la valeur filtrée.
    uint8_t DSP_conf_mask = 0x2B; // Lecture après filtre IIR (P+T) + Compensation ON

    // Mode CONTINU (Le capteur tourne tout seul en boucle)
    uint8_t ODR_mask      = 0x03;

    // =========================================================================
    // 2. ÉCRITURE DES CONFIGURATIONS (LE CAPTEUR DOIT ÊTRE EN STANDBY)
    // =========================================================================
    if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_OSR_CONFIG, 1, &OSR_mask, 1, 100) != HAL_OK) check = 1;
    if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, 0x31 /* DSP_IIR */, 1, &DSP_IIR_mask, 1, 100) != HAL_OK) check = 1;
    if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_DSP_CONFIG, 1, &DSP_conf_mask, 1, 100) != HAL_OK) check = 1;
    // =========================================================================
    // 3. LE DÉCLENCHEUR : On réveille le capteur SEULEMENT quand tout est prêt
    // =========================================================================
    if(HAL_I2C_Mem_Write(&hi2c1, BMP581_WRITE_ADDR, BMP581_ODR_CONFIG, 1, &ODR_mask, 1, 100) != HAL_OK) check = 1;

    // 4. Vérification pour s'assurer que le capteur est vivant
    if(HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, BMP581_OSR_EFF, 1, &odrcheck, 1, 100) != HAL_OK) check = 1;

    return check;
}
/* Previous code :
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
*/
uint8_t bmp581_read_precise_normal(BMP_t * bmp581){
		// Polling manuel (capteur en continu, pas d'interruption) : on lit directement
		// les registres de données, pas besoin de passer par INT_STATUS.
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
// Back to a short fixed duration: the adaptive thermal-stability wait (kept
// running until the die stopped drifting, up to 90s) could leave the HMI
// screen looking frozen for a very long time when run right after config
// entry, since the board is mostly idle there and may drift very slowly
// without ever crossing the stability threshold. Triggering this at CONFIG
// EXIT instead (see startTaskFSM) — after the operator has been sitting in
// config for a while — is the cheaper fix to try first.
#define BARO_CALIB_SAMPLE_DELAY_MS   60
#define BARO_CALIB_DURATION_MS       8000
#define BARO_CALIB_TOTAL_SAMPLES     (BARO_CALIB_DURATION_MS / BARO_CALIB_SAMPLE_DELAY_MS)

/**
 * @brief  Takes multiple readings to establish the baseline ground pressure.
 *         Fixed ~8s duration; only the second half is averaged to let any
 *         short transient from waking the sensor settle.
 * @param  reference_pressure: Pointer to double where the average pressure (Pa) will be stored.
 * @param  reference_temp: Pointer to double where the average temperature (°C) will be stored.
 * @param  progress_cb: Optional callback invoked periodically with 0-100 (percent of
 *         the fixed duration elapsed). May be NULL.
 * @retval HAL_OK if successful, HAL_ERROR if the sensor never produced a valid read.
 */
HAL_StatusTypeDef BMP581_CalibrateGroundPressure(double *reference_pressure, double *reference_temp, BMP_t * bmp581,
                                                  void (*progress_cb)(uint8_t percent)) {
    extern UART_HandleTypeDef huart1;
    extern osMutexId_t I2C1_MutexHandle;
    char msg[64];

    extern double bmppress;
    extern double bmptemp;

    double sum_pressure = 0.0;
    double sum_temp = 0.0;
    int valid_reads = 0;

    int discard_count = BARO_CALIB_TOTAL_SAMPLES / 2;
    for (int i = 0; i < BARO_CALIB_TOTAL_SAMPLES; i++) {
        // Mutex taken per-sample (not held for the whole call) so the HMI
        // task can still get at the SSD1306 on I2C1 between reads.
        if (osMutexAcquire(I2C1_MutexHandle, osWaitForever) == osOK) {
            uint8_t read_ok = (bmp581_read_precise_normal(bmp581) == 0);
            osMutexRelease(I2C1_MutexHandle);

            if (read_ok && i >= discard_count) {
                sum_pressure += bmppress;
                sum_temp += bmptemp;
                valid_reads++;
            }
        }

        if (i % 20 == 0) {
            uint8_t percent = (uint8_t)((i * 100) / BARO_CALIB_TOTAL_SAMPLES);
            sprintf(msg, "[*] Calibrating... %d%%\r\n", percent);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            if (progress_cb) progress_cb(percent);
        }
        osDelay(BARO_CALIB_SAMPLE_DELAY_MS);
    }

    if (valid_reads > 0) {
        *reference_pressure = sum_pressure / (double)valid_reads;
        *reference_temp = sum_temp / (double)valid_reads;
        if (progress_cb) progress_cb(100);
        return HAL_OK;
    }
    return HAL_ERROR;
}
