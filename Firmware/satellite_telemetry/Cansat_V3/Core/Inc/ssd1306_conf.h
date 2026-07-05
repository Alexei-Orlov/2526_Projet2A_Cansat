#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

#include "stm32g4xx_hal.h"

/* Communication mode — I2C only on this board */
#define SSD1306_USE_I2C

/* SBC-OLED01 screen dimensions */
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   64

/* I2C handle — must match main.c */
extern I2C_HandleTypeDef hi2c1;
#define SSD1306_I2C_PORT    hi2c1
#define SSD1306_I2C_ADDR    (0x3C << 1)  /* 0x78 */

/* I2C timeout (ms) */
#define SSD1306_I2C_TIMEOUT 100

/* Available fonts — uncomment as needed */
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

/* Optional display transformations */
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ
// #define SSD1306_INVERSE_COLOR

#endif /* __SSD1306_CONF_H__ */
