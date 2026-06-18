/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr55xx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "transport_scheduler.h"
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"
#include "gus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "audio_capture.h"
#include "audio_storage.h"
#include "audio_file_transfer.h"
#include "record_key.h"
#include "oled_ssd1315.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern gap_cb_fun_t         app_gap_callbacks;
extern gatt_common_cb_fun_t app_gatt_common_callback;

#define AUDIO_TASK_STACK_SIZE      768u
#define SYSTEM_TASK_STACK_SIZE     512u
#define START_TASK_STACK_SIZE      512u
#define AUDIO_TASK_PRIORITY        (configMAX_PRIORITIES - 1)
#define SYSTEM_TASK_PRIORITY       (configMAX_PRIORITIES - 3)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

volatile uint32_t g_debug_rtos_task_create_status;
volatile uint8_t g_debug_audio_storage_init_status;

static app_callback_t s_app_ble_callback =
{
    .app_ble_init_cmp_callback = ble_init_cmp_callback,
    .app_gap_callbacks         = &app_gap_callbacks,
    .app_gatt_common_callback  = &app_gatt_common_callback,
    .app_gattc_callback        = NULL,
    .app_sec_callback          = NULL,
};

static void audio_task(void *p_arg)
{
    (void)p_arg;

    for (;;)
    {
        record_key_process();
        audio_capture_process();
        audio_storage_process();

        // 录音期间也 yield，让 system_task 能调度 BLE
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void system_task(void *p_arg)
{
    (void)p_arg;

    for (;;)
    {
        if (!audio_capture_is_active() && !audio_storage_is_recording())
        {
            app_log_flush();
            pwr_mgmt_schedule();
            transport_schedule();      // 非录音时在system_task调度
            audio_file_transfer_process();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void start_task(void *p_arg)
{
    BaseType_t ret;

    (void)p_arg;

    audio_hardware_init();
    g_debug_audio_storage_init_status = audio_storage_init();
    oled_ssd1315_init();
    record_key_init();

    ret = xTaskCreate(audio_task, "audio_task", AUDIO_TASK_STACK_SIZE, NULL, AUDIO_TASK_PRIORITY, NULL);
    if (ret != pdPASS)
    {
        g_debug_rtos_task_create_status |= 0x01u;
    }

    ret = xTaskCreate(system_task, "system_task", SYSTEM_TASK_STACK_SIZE, NULL, SYSTEM_TASK_PRIORITY, NULL);
    if (ret != pdPASS)
    {
        g_debug_rtos_task_create_status |= 0x02u;
    }

    vTaskDelete(NULL);
}

int main(void)
{
    // Initialize user peripherals.
    app_periph_init();
    
    // Initialize ble stack.
    ble_stack_init(&s_app_ble_callback, &heaps_table);

    xTaskCreate(start_task, "start_task", START_TASK_STACK_SIZE, NULL, 0, NULL);
    vTaskStartScheduler();

    for (;;);
}
