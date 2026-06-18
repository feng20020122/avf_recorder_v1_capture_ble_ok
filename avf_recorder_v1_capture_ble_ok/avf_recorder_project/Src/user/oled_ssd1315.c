#include "oled_ssd1315.h"

#include "app_i2c.h"
#include "app_io.h"
#include "gr55xx_delay.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define OLED_IO_TYPE             APP_IO_TYPE_MSIO
#define OLED_SCL_PIN             APP_IO_PIN_0
#define OLED_SDA_PIN             APP_IO_PIN_1
#define OLED_SW_PIN              APP_IO_PIN_2
#define OLED_RST_PIN             APP_IO_PIN_3

#define OLED_I2C_ID              APP_I2C_ID_1
#define OLED_I2C_ADDR_7BIT       0x3Cu
#define OLED_I2C_TIMEOUT_MS      100u
#define OLED_CTRL_COMMAND        0x00u
#define OLED_CTRL_DATA           0x40u
#define OLED_COL_LOW             0x02u
#define OLED_COL_HIGH            0x12u

volatile uint8_t  g_debug_oled_init_status = OLED_SSD1315_STATUS_NOT_INIT;

static bool s_oled_initialized;

static const uint8_t s_digit_font[3][5] =
{
    {0x00u, 0x42u, 0x7Fu, 0x40u, 0x00u},
    {0x62u, 0x51u, 0x49u, 0x49u, 0x46u},
    {0x22u, 0x41u, 0x49u, 0x49u, 0x36u},
};

static const uint8_t s_work_font[4][5] =
{
    {0x7Fu, 0x20u, 0x18u, 0x20u, 0x7Fu},
    {0x3Eu, 0x41u, 0x41u, 0x41u, 0x3Eu},
    {0x7Fu, 0x09u, 0x19u, 0x29u, 0x46u},
    {0x7Fu, 0x08u, 0x14u, 0x22u, 0x41u},
};

static void oled_io_config(uint32_t pin, app_io_mode_t mode, app_io_pull_t pull)
{
    app_io_init_t io_cfg = APP_IO_DEFAULT_CONFIG;

    io_cfg.pin  = pin;
    io_cfg.mode = mode;
    io_cfg.pull = pull;
    io_cfg.mux  = APP_IO_MUX_7;
    (void)app_io_init(OLED_IO_TYPE, &io_cfg);
}

static bool oled_i2c_bus_init(void)
{
    app_i2c_params_t i2c_params;
    uint16_t ret;

    memset(&i2c_params, 0, sizeof(i2c_params));

    i2c_params.id = OLED_I2C_ID;
    i2c_params.role = APP_I2C_ROLE_MASTER;
    i2c_params.pin_cfg.scl.type = OLED_IO_TYPE;
    i2c_params.pin_cfg.scl.mux  = APP_IO_MUX_4;
    i2c_params.pin_cfg.scl.pin  = OLED_SCL_PIN;
    i2c_params.pin_cfg.scl.pull = APP_IO_PULLUP;
    i2c_params.pin_cfg.sda.type = OLED_IO_TYPE;
    i2c_params.pin_cfg.sda.mux  = APP_IO_MUX_4;
    i2c_params.pin_cfg.sda.pin  = OLED_SDA_PIN;
    i2c_params.pin_cfg.sda.pull = APP_IO_PULLUP;
    i2c_params.use_mode.type    = APP_I2C_TYPE_POLLING;
    i2c_params.init.speed       = I2C_SPEED_100K;
    i2c_params.init.own_address = 0;
    i2c_params.init.addressing_mode = I2C_ADDRESSINGMODE_7BIT;

    ret = app_i2c_init(&i2c_params, NULL);
    if (ret != APP_DRV_SUCCESS)
    {
        return false;
    }

    return true;
}

static bool oled_i2c_write(uint8_t *data, uint16_t size)
{
    uint16_t ret = app_i2c_transmit_sync(OLED_I2C_ID, OLED_I2C_ADDR_7BIT, data, size, OLED_I2C_TIMEOUT_MS);

    if (ret == APP_DRV_SUCCESS)
    {
        return true;
    }

    return false;
}

static bool oled_write_command(uint8_t cmd)
{
    uint8_t data[2];

    data[0] = OLED_CTRL_COMMAND;
    data[1] = cmd;

    return oled_i2c_write(data, sizeof(data));
}

static bool oled_write_command2(uint8_t cmd, uint8_t arg)
{
    uint8_t data[3];

    data[0] = OLED_CTRL_COMMAND;
    data[1] = cmd;
    data[2] = arg;

    return oled_i2c_write(data, sizeof(data));
}

static bool oled_set_page_column(uint8_t page)
{
    bool ok = true;

    ok &= oled_write_command((uint8_t)(0xB0u + page));
    ok &= oled_write_command(OLED_COL_LOW);
    ok &= oled_write_command(OLED_COL_HIGH);

    return ok;
}

static bool oled_write_page_pattern(uint8_t page, uint8_t pattern, bool stripe)
{
    bool ok;
    uint8_t data[OLED_SSD1315_WIDTH + 1u];

    ok = oled_set_page_column(page);
    data[0] = OLED_CTRL_DATA;
    for (uint8_t col = 0; col < OLED_SSD1315_WIDTH; col++)
    {
        data[col + 1u] = stripe ? (((col / 6u) & 1u) ? 0xFFu : 0x00u) : pattern;
    }
    ok &= oled_i2c_write(data, sizeof(data));

    return ok;
}

static bool oled_write_buffer(const uint8_t *buffer)
{
    bool ok = true;

    for (uint8_t page = 0; page < OLED_SSD1315_PAGES; page++)
    {
        uint8_t data[OLED_SSD1315_WIDTH + 1u];

        ok &= oled_set_page_column(page);
        data[0] = OLED_CTRL_DATA;
        memcpy(&data[1], &buffer[page * OLED_SSD1315_WIDTH], OLED_SSD1315_WIDTH);
        ok &= oled_i2c_write(data, sizeof(data));
    }

    return ok;
}

static void oled_draw_scaled_glyph(uint8_t *buffer,
                                   const uint8_t glyph[5],
                                   uint8_t x,
                                   uint8_t y,
                                   uint8_t scale)
{
    for (uint8_t src_x = 0; src_x < 5u; src_x++)
    {
        for (uint8_t src_y = 0; src_y < 7u; src_y++)
        {
            if ((glyph[src_x] & (1u << src_y)) == 0u)
            {
                continue;
            }

            for (uint8_t dx = 0; dx < scale; dx++)
            {
                for (uint8_t dy = 0; dy < scale; dy++)
                {
                    uint8_t px = x + src_x * scale + dx;
                    uint8_t py = y + src_y * scale + dy;

                    if (px < OLED_SSD1315_WIDTH && py < OLED_SSD1315_HEIGHT)
                    {
                        buffer[(py / 8u) * OLED_SSD1315_WIDTH + px] |= (uint8_t)(1u << (py % 8u));
                    }
                }
            }
        }
    }
}

void oled_ssd1315_power_on(void)
{
    oled_io_config(OLED_SW_PIN, APP_IO_MODE_OUT_PUT, APP_IO_NOPULL);
    app_io_write_pin(OLED_IO_TYPE, OLED_SW_PIN, APP_IO_PIN_SET);
    delay_ms(20);
}

void oled_ssd1315_power_off(void)
{
    oled_ssd1315_set_display_on(false);
    app_io_write_pin(OLED_IO_TYPE, OLED_SW_PIN, APP_IO_PIN_RESET);
    s_oled_initialized = false;
    g_debug_oled_init_status = OLED_SSD1315_STATUS_NOT_INIT;
}

void oled_ssd1315_set_display_on(bool on)
{
    (void)oled_write_command(on ? 0xAFu : 0xAEu);
}

void oled_ssd1315_clear(void)
{
    bool ok = true;

    for (uint8_t page = 0; page < OLED_SSD1315_PAGES; page++)
    {
        ok &= oled_write_page_pattern(page, 0x00u, false);
    }

    if (!ok)
    {
        g_debug_oled_init_status = OLED_SSD1315_STATUS_I2C_ERROR;
    }
}

void oled_ssd1315_fill(uint8_t pattern)
{
    bool ok = true;

    for (uint8_t page = 0; page < OLED_SSD1315_PAGES; page++)
    {
        ok &= oled_write_page_pattern(page, pattern, false);
    }

    if (!ok)
    {
        g_debug_oled_init_status = OLED_SSD1315_STATUS_I2C_ERROR;
    }
}

void oled_ssd1315_show_test_pattern(void)
{
    bool ok = s_oled_initialized;

    for (uint8_t page = 0; page < OLED_SSD1315_PAGES; page++)
    {
        ok &= oled_write_page_pattern(page, 0x00u, true);
    }

    if (!ok)
    {
        g_debug_oled_init_status = OLED_SSD1315_STATUS_I2C_ERROR;
    }
}

void oled_ssd1315_show_position(uint8_t position)
{
    uint8_t buffer[OLED_SSD1315_WIDTH * OLED_SSD1315_PAGES];
    bool ok;

    if (position < 1u || position > 3u)
    {
        position = 1u;
    }

    memset(buffer, 0, sizeof(buffer));
    oled_draw_scaled_glyph(buffer, s_digit_font[position - 1u], 20u, 2u, 4u);

    ok = s_oled_initialized && oled_write_buffer(buffer);
    if (!ok)
    {
        g_debug_oled_init_status = OLED_SSD1315_STATUS_I2C_ERROR;
    }
}

void oled_ssd1315_show_working(void)
{
    uint8_t buffer[OLED_SSD1315_WIDTH * OLED_SSD1315_PAGES];
    bool ok;
    uint8_t x = 2u;

    memset(buffer, 0, sizeof(buffer));
    for (uint8_t i = 0; i < 4u; i++)
    {
        oled_draw_scaled_glyph(buffer, s_work_font[i], x, 9u, 2u);
        x = (uint8_t)(x + 14u);
    }

    ok = s_oled_initialized && oled_write_buffer(buffer);
    if (!ok)
    {
        g_debug_oled_init_status = OLED_SSD1315_STATUS_I2C_ERROR;
    }
}

void oled_ssd1315_init(void)
{
    bool ok = true;

    s_oled_initialized = false;

    oled_io_config(OLED_SW_PIN, APP_IO_MODE_OUT_PUT, APP_IO_NOPULL);
    oled_io_config(OLED_RST_PIN, APP_IO_MODE_OUT_PUT, APP_IO_NOPULL);

    app_io_write_pin(OLED_IO_TYPE, OLED_SW_PIN, APP_IO_PIN_RESET);
    app_io_write_pin(OLED_IO_TYPE, OLED_RST_PIN, APP_IO_PIN_RESET);
    delay_ms(5);

    ok &= oled_i2c_bus_init();

    app_io_write_pin(OLED_IO_TYPE, OLED_RST_PIN, APP_IO_PIN_SET);
    delay_ms(20);

    ok &= oled_write_command(0xAEu);
    ok &= oled_write_command(0x02u);
    ok &= oled_write_command(0x12u);
    ok &= oled_write_command(0x00u);
    ok &= oled_write_command(0xB0u);
    ok &= oled_write_command2(0x81u, 0x18u);
    ok &= oled_write_command(0xA1u);
    ok &= oled_write_command(0xA6u);
    ok &= oled_write_command2(0xA8u, 0x1Fu);
    ok &= oled_write_command(0xC8u);
    ok &= oled_write_command2(0xD3u, 0x00u);
    ok &= oled_write_command2(0xD5u, 0x80u);
    ok &= oled_write_command2(0xD9u, 0xF1u);
    ok &= oled_write_command2(0xDAu, 0x12u);
    ok &= oled_write_command2(0xDBu, 0x40u);
    ok &= oled_write_command2(0x8Du, 0x14u);
    ok &= oled_write_command2(0xADu, 0x30u);
    ok &= oled_write_command(0xA4u);

    s_oled_initialized = ok;
    oled_ssd1315_clear();

    oled_ssd1315_power_on();
    delay_ms(100);

    ok &= oled_write_command(0xAFu);

    s_oled_initialized = ok;
    g_debug_oled_init_status = ok ? OLED_SSD1315_STATUS_OK : OLED_SSD1315_STATUS_I2C_ERROR;
}
