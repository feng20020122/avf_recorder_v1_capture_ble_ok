#ifndef OLED_SSD1315_H
#define OLED_SSD1315_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_SSD1315_WIDTH       60u
#define OLED_SSD1315_HEIGHT      32u
#define OLED_SSD1315_PAGES       (OLED_SSD1315_HEIGHT / 8u)

typedef enum
{
    OLED_SSD1315_STATUS_OK = 0,
    OLED_SSD1315_STATUS_NOT_INIT = 1,
    OLED_SSD1315_STATUS_I2C_ERROR = 2,
} oled_ssd1315_status_t;

extern volatile uint8_t  g_debug_oled_init_status;

void oled_ssd1315_init(void);
void oled_ssd1315_power_on(void);
void oled_ssd1315_power_off(void);
void oled_ssd1315_clear(void);
void oled_ssd1315_fill(uint8_t pattern);
void oled_ssd1315_show_test_pattern(void);
void oled_ssd1315_show_position(uint8_t position);
void oled_ssd1315_show_working(void);
void oled_ssd1315_set_display_on(bool on);

#endif
