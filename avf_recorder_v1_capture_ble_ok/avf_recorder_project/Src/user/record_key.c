#include "record_key.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_gpiote.h"
#include "app_io.h"
#include "app_log.h"
#include "audio_storage.h"
#include "oled_ssd1315.h"
#include "transport_scheduler.h"
#include "audio_capture.h"

#define RECORD_KEY_TYPE             APP_IO_TYPE_NORMAL
#define RECORD_KEY_PIN              APP_IO_PIN_5
#define RECORD_KEY_DEBOUNCE_MS      30u
#define RECORD_KEY_LONG_PRESS_MS    2000u
#define RECORD_DURATION_SECONDS     10u
#define RECORD_FORCE_TIMEOUT_MS     (RECORD_DURATION_SECONDS * 3000u)
#define RECORD_TARGET_SAMPLES       (AUDIO_SAMPLE_RATE * RECORD_DURATION_SECONDS)
#define RECORD_POSITION_COUNT       3u

static volatile bool s_key_irq_pending;
static volatile uint32_t s_key_irq_cycles;
static bool s_key_pressed;
static bool s_key_press_handled;
static uint32_t s_key_press_cycles;
static bool s_record_timer_running;
static uint32_t s_record_start_cycles;
static uint8_t s_record_position = 1u;
static uint16_t s_position_file_index[RECORD_POSITION_COUNT];

volatile uint32_t g_debug_key_press_count;
volatile uint8_t g_debug_key_last_start_status;
volatile uint32_t g_debug_record_elapsed_ms;
volatile uint8_t g_debug_record_position = 1u;

static void record_timebase_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t record_elapsed_ms_get(uint32_t start_cycles)
{
    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    uint32_t cycles_per_ms = SystemCoreClock / 1000u;

    if (cycles_per_ms == 0u)
    {
        return 0u;
    }

    return elapsed_cycles / cycles_per_ms;
}

static void record_key_irq_callback(app_gpiote_evt_t *p_evt)
{
    if (p_evt == NULL)
    {
        return;
    }

    if (p_evt->type == RECORD_KEY_TYPE && p_evt->pin == RECORD_KEY_PIN)
    {
        s_key_irq_cycles = DWT->CYCCNT;
        s_key_irq_pending = true;
    }
}

static void record_file_name_make(char *p_name, uint8_t position, uint16_t index)
{
    if (position < 1u || position > RECORD_POSITION_COUNT)
    {
        position = 1u;
    }

    sprintf(p_name, "P%u_%03u.WAV", position, index);
}

static void record_start(void)
{
    audio_storage_status_t status;
    char file_name[13];
    uint16_t index;
    uint16_t attempts = 0;

    if (audio_storage_is_recording())
    {
        return;
    }

    do
    {
        index = s_position_file_index[s_record_position - 1u] + 1u;
        if (index > 999u)
        {
            index = 1u;
        }
        s_position_file_index[s_record_position - 1u] = index;
        record_file_name_make(file_name, s_record_position, index);

        status = audio_storage_start_wav_for_samples(AUDIO_SAMPLE_RATE, 16, 1, RECORD_TARGET_SAMPLES, file_name);
        if (status == AUDIO_STORAGE_ERR_EXISTS)
        {
            attempts++;
        }
    }
    while (status == AUDIO_STORAGE_ERR_EXISTS && attempts < 999u);

    if (status != AUDIO_STORAGE_OK)
    {
        (void)audio_storage_init();
        status = audio_storage_start_wav_for_samples(AUDIO_SAMPLE_RATE, 16, 1, RECORD_TARGET_SAMPLES, file_name);
    }

    g_debug_key_last_start_status = status;

    if (status == AUDIO_STORAGE_OK)
    {
        uint16_t i2s_status = audio_capture_start();

        if (i2s_status == APP_DRV_SUCCESS)
        {
            s_record_start_cycles = DWT->CYCCNT;
            g_debug_record_elapsed_ms = 0;
            s_record_timer_running = true;
            oled_ssd1315_show_working();
            APP_LOG_INFO("KEY start recording %u seconds.", RECORD_DURATION_SECONDS);
        }
        else
        {
            audio_storage_stop();
            status = AUDIO_STORAGE_ERR_NOT_READY;
            g_debug_key_last_start_status = status;
            APP_LOG_ERROR("KEY failed to start I2S capture.");
        }
    }
    else
    {
        APP_LOG_ERROR("KEY failed to start recording.");
    }
}

static void record_position_next(void)
{
    if (audio_storage_is_recording() || s_record_timer_running)
    {
        return;
    }

    s_record_position++;
    if (s_record_position > RECORD_POSITION_COUNT)
    {
        s_record_position = 1u;
    }

    g_debug_record_position = s_record_position;
    oled_ssd1315_show_position(s_record_position);
}

void record_key_init(void)
{
    app_gpiote_param_t key_cfg;

    memset(&key_cfg, 0, sizeof(key_cfg));
    record_timebase_init();

    key_cfg.type = RECORD_KEY_TYPE;
    key_cfg.pin = RECORD_KEY_PIN;
    key_cfg.mode = APP_IO_MODE_IT_FALLING;
    key_cfg.pull = APP_IO_PULLUP;
    key_cfg.handle_mode = APP_IO_DISABLE_WAKEUP;
    key_cfg.io_evt_cb = record_key_irq_callback;

    (void)app_gpiote_init(&key_cfg, 1);

    s_key_irq_pending = false;
    s_key_irq_cycles = 0;
    s_key_pressed = false;
    s_key_press_handled = false;
    s_key_press_cycles = 0;
    s_record_timer_running = false;
    s_record_position = 1u;
    memset(s_position_file_index, 0, sizeof(s_position_file_index));
    g_debug_record_position = s_record_position;
    oled_ssd1315_show_position(s_record_position);
}

void record_key_process(void)
{
    if (s_record_timer_running)
    {
        g_debug_record_elapsed_ms = record_elapsed_ms_get(s_record_start_cycles);
        if (audio_storage_is_capture_target_reached())
        {
            audio_capture_stop();
            s_record_timer_running = false;
            oled_ssd1315_show_position(s_record_position);
        }
        else if (g_debug_record_elapsed_ms >= RECORD_FORCE_TIMEOUT_MS)
        {
            audio_capture_stop();
            audio_storage_capture_done();
            s_record_timer_running = false;
            oled_ssd1315_show_position(s_record_position);
        }
    }

    if (!s_key_pressed)
    {
        if (!s_key_irq_pending)
        {
            return;
        }

        if (record_elapsed_ms_get(s_key_irq_cycles) < RECORD_KEY_DEBOUNCE_MS)
        {
            return;
        }

        s_key_irq_pending = false;
        if (app_io_read_pin(RECORD_KEY_TYPE, RECORD_KEY_PIN) != APP_IO_PIN_RESET)
        {
            return;
        }

        s_key_pressed = true;
        s_key_press_handled = false;
        s_key_press_cycles = s_key_irq_cycles;
        return;
    }

    if (app_io_read_pin(RECORD_KEY_TYPE, RECORD_KEY_PIN) == APP_IO_PIN_RESET)
    {
        if (!s_key_press_handled &&
            record_elapsed_ms_get(s_key_press_cycles) >= RECORD_KEY_LONG_PRESS_MS)
        {
            s_key_press_handled = true;
            g_debug_key_press_count++;
            record_start();
        }
        return;
    }

    if (record_elapsed_ms_get(s_key_press_cycles) < RECORD_KEY_DEBOUNCE_MS)
    {
        return;
    }

    if (!s_key_press_handled)
    {
        g_debug_key_press_count++;
        record_position_next();
    }

    s_key_pressed = false;
    s_key_press_handled = false;
}
