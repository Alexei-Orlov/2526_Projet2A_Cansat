#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

/* Inclure le HAL de ta carte */
#include "stm32g4xx_hal.h"

/* Mode de communication — I2C obligatoire ici */
#define SSD1306_USE_I2C

/* Taille de l'écran SBC-OLED01 */
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64

/* Handle I2C — doit correspondre à ce qui est dans main.c */
extern I2C_HandleTypeDef hi2c1;
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1)   // 0x78

/* Timeout I2C en ms */
#define SSD1306_I2C_TIMEOUT     100

/* Fonts disponibles — décommenter celles dont tu as besoin */
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

/* Optionnel */
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ
// #define SSD1306_INVERSE_COLOR

#endif /* __SSD1306_CONF_H__ */
