/* ===========================================================
 * Test écran SBC-OLED01 (SSD1306) via I2C1
 * Nucléo-L476RG — STM32CubeIDE HAL
 *
 * Connexions :
 *   SDA  → PB7 (CN10 pin 21)
 *   SCL  → PB6 (CN10 pin 17)
 *   VCC  → 3.3V
 *   GND  → GND
 *
 * Bibliothèque requise :
 *   https://github.com/afiskon/stm32-ssd1306
 *   → copier ssd1306.c/.h + ssd1306_fonts.c/.h dans Drivers/SSD1306/
 * =========================================================== */

#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* Handle I2C généré par CubeIDE — vérifier que c'est bien hi2c1 dans main.c */
extern I2C_HandleTypeDef hi2c1;

/* -----------------------------------------------------------
 * Adresse I2C du SBC-OLED01 :
 *   - 0x3C par défaut (la plus courante)
 *   - 0x3D si la résistance de config est changée
 * Dans ssd1306.h, vérifier/modifier :
 *   #define SSD1306_I2C_ADDR  0x3C
 * ----------------------------------------------------------- */

void SSD1306_Test(void)
{
    /* 1. Initialisation de l'écran */
    ssd1306_Init();

    /* -------------------------------------------------------
     * TEST 1 : Texte simple
     * ------------------------------------------------------- */
    ssd1306_Fill(Black);                          // Efface l'écran (fond noir)

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("CanSat IHM", Font_11x18, White);

    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("Test ecran OK", Font_7x10, White);

    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("SBC-OLED01", Font_7x10, White);

    ssd1306_UpdateScreen();                       // Envoie le buffer sur l'écran
    HAL_Delay(2000);

    /* -------------------------------------------------------
     * TEST 2 : Affichage de plusieurs pages
     * (simule la navigation bouton)
     * ------------------------------------------------------- */

    /* Page 1 : État LoRa */
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Page 1/3 --", Font_7x10, White);
    ssd1306_SetCursor(0, 18);
    ssd1306_WriteString("LoRa :", Font_7x10, White);
    ssd1306_SetCursor(50, 18);
    ssd1306_WriteString("ACTIF", Font_7x10, White);
    ssd1306_SetCursor(0, 36);
    ssd1306_WriteString("Freq: 868 MHz", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);

    /* Page 2 : État capteurs */
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Page 2/3 --", Font_7x10, White);
    ssd1306_SetCursor(0, 18);
    ssd1306_WriteString("Baro  : OK", Font_7x10, White);
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("IMU   : OK", Font_7x10, White);
    ssd1306_SetCursor(0, 42);
    ssd1306_WriteString("GPS   : OK", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);

    /* Page 3 : Tension batterie */
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("-- Page 3/3 --", Font_7x10, White);
    ssd1306_SetCursor(0, 18);
    ssd1306_WriteString("Batterie :", Font_7x10, White);
    ssd1306_SetCursor(0, 36);
    ssd1306_WriteString("7.4V  100%", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);

    /* -------------------------------------------------------
     * TEST 3 : Lignes et formes géométriques
     * ------------------------------------------------------- */
    ssd1306_Fill(Black);
    /* Cadre autour de l'écran */
    ssd1306_DrawRectangle(0, 0, 127, 63, White);
    /* Croix au centre */
    ssd1306_Line(0, 32, 127, 32, White);
    ssd1306_Line(64, 0, 64, 63, White);
    /* Cercle au centre */
    ssd1306_DrawCircle(64, 32, 20, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);

    /* -------------------------------------------------------
     * TEST 4 : Inversion de l'affichage
     * ------------------------------------------------------- */
    ssd1306_Fill(White);
    ssd1306_SetCursor(10, 24);
    ssd1306_WriteString("INVERSE !", Font_11x18, Black);
    ssd1306_UpdateScreen();
    HAL_Delay(1500);

    /* Retour normal */
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 24);
    ssd1306_WriteString("NORMAL", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(1500);

    /* -------------------------------------------------------
     * Écran final : prêt
     * ------------------------------------------------------- */
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("CanSat ready !", Font_7x10, White);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString("Appuyer sur BTN", Font_7x10, White);
    ssd1306_SetCursor(0, 35);
    ssd1306_WriteString("pour naviguer", Font_7x10, White);
    ssd1306_UpdateScreen();
}

/* -----------------------------------------------------------
 * Intégration dans main.c
 *
 * Dans le main(), après MX_I2C1_Init() :
 *
 *   SSD1306_Test();
 *
 *   // Puis dans la boucle while(1), tu pourras ajouter
 *   // la navigation au bouton (prochaine étape)
 * ----------------------------------------------------------- */
