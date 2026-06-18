#include "wm8960_inmp441.h"
#include "app_log.h"
#include "gr55xx_delay.h"
#include <string.h>

/* I2C 寮曡剼閰嶇疆锛岃鏍规嵁瀹為檯纭欢杩炴帴淇敼 */
#define I2C_SCL_PIN      APP_IO_PIN_30
#define I2C_SDA_PIN      APP_IO_PIN_26
#define I2C_SCL_MUX      APP_IO_MUX_2
#define I2C_SDA_MUX      APP_IO_MUX_2

/* I2S_M 寮曡剼閰嶇疆(Master)锛岃鏍规嵁瀹為檯纭欢杩炴帴淇敼 */
#define I2S_WS_PIN       APP_IO_PIN_24
#define I2S_SCLK_PIN     APP_IO_PIN_17
#define I2S_TX_PIN       APP_IO_PIN_25 // -> WM8960 DACDAT
#define I2S_RX_PIN       APP_IO_PIN_16 // <- INMP441 SD
#define I2S_MUX          APP_IO_MUX_3

static uint8_t g_wm8960_addr = WM8960_I2C_ADDR_1;

static int16_t rx_buffer[AUDIO_FRAME_SAMPLES * 2];
static int16_t tx_buffer[AUDIO_FRAME_SAMPLES * 2];

static float dc_block_x = 0.0f;
static float dc_block_y = 0.0f;

static bool wm8960_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t data[2];
    data[0] = (reg << 1) | ((val >> 8) & 0x01);
    data[1] = val & 0xFF;
    
    uint16_t ret = app_i2c_transmit_sync(APP_I2C_ID_0, g_wm8960_addr, data, 2, 100);
    return (ret == APP_DRV_SUCCESS);
}

static bool wm8960_write_reg_checked(uint8_t reg, uint16_t val, const char* name)
{
    bool ok = wm8960_write_reg(reg, val);
    if (!ok) {
        APP_LOG_ERROR("WM8960 reg write failed: %s (0x%02X)=0x%03X", name, reg, val);
    }
    return ok;
}

static bool wm8960_probe(void)
{
    uint8_t addrs[2] = {WM8960_I2C_ADDR_1, WM8960_I2C_ADDR_2};
    uint8_t dummy = 0;
    
    for (int i = 0; i < 2; i++) {
        g_wm8960_addr = addrs[i];
        if (app_i2c_transmit_sync(APP_I2C_ID_0, g_wm8960_addr, &dummy, 0, 100) == APP_DRV_SUCCESS) {
            return true;
        }
    }
    return false;
}

static bool wm8960_init(void)
{
    if (!wm8960_write_reg_checked(0x0F, 0x000, "RESET")) return false;
    delay_ms(20);

    /* 鐢垫簮涓庨€氳矾鍒濆鍖?*/
    if (!wm8960_write_reg_checked(0x19, 0x0FC, "PWR1")) return false; 
    if (!wm8960_write_reg_checked(0x1A, 0x1F8, "PWR2")) return false; 
    if (!wm8960_write_reg_checked(0x2F, 0x03C, "PWR3")) return false; 

    /* I2S 浠庢満锛?6-bit锛孖2S鏍煎紡 */
    if (!wm8960_write_reg_checked(0x04, 0x000, "CLOCK1")) return false;
    if (!wm8960_write_reg_checked(0x07, 0x002, "IFACE1")) return false;

    /* 鍙栨秷 DAC 闈欓煶 */
    if (!wm8960_write_reg_checked(0x05, 0x000, "ADCDAC")) return false;

    /* DAC 鏁板瓧闊抽噺 */
    if (!wm8960_write_reg_checked(0x0A, 0x1FF, "LDAC")) return false;
    if (!wm8960_write_reg_checked(0x0B, 0x1FF, "RDAC")) return false;

    /* 鑰虫満杈撳嚭闊抽噺 */
    if (!wm8960_write_reg_checked(0x02, 0x179, "LOUT1")) return false;
    if (!wm8960_write_reg_checked(0x03, 0x179, "ROUT1")) return false;

    /* 灏嗗乏鍙?DAC 缁熶竴娣峰埌鑰虫満 */
    if (!wm8960_write_reg_checked(0x22, 0x1B0, "LOUTMIX")) return false;
    if (!wm8960_write_reg_checked(0x25, 0x1B0, "ROUTMIX")) return false;

    /* 鎶楀暤闊宠缃?*/
    if (!wm8960_write_reg_checked(0x1C, 0x09F, "ANTI_POP1")) return false;
    if (!wm8960_write_reg_checked(0x1D, 0x000, "ANTI_POP2")) return false;

    return true;
}

static void i2c_bus_init(void)
{
    app_i2c_params_t i2c_params;
    i2c_params.id = APP_I2C_ID_0;
    i2c_params.role = APP_I2C_ROLE_MASTER;
    i2c_params.pin_cfg.scl.type = APP_IO_TYPE_NORMAL;
    i2c_params.pin_cfg.scl.mux  = I2C_SCL_MUX;
    i2c_params.pin_cfg.scl.pin  = I2C_SCL_PIN;
    i2c_params.pin_cfg.scl.pull = APP_IO_PULLUP;
    i2c_params.pin_cfg.sda.type = APP_IO_TYPE_NORMAL;
    i2c_params.pin_cfg.sda.mux  = I2C_SDA_MUX;
    i2c_params.pin_cfg.sda.pin  = I2C_SDA_PIN;
    i2c_params.pin_cfg.sda.pull = APP_IO_PULLUP;
    i2c_params.use_mode.type = APP_I2C_TYPE_POLLING;
    i2c_params.init.speed = I2C_SPEED_400K;

    app_i2c_init(&i2c_params, NULL);
}

static void i2s_bus_init(void)
{
    app_i2s_params_t i2s_params;
    memset(&i2s_params, 0, sizeof(app_i2s_params_t));
    i2s_params.id = APP_I2S_ID_MASTER;

    i2s_params.pin_cfg.sclk.type = APP_IO_TYPE_NORMAL;
    i2s_params.pin_cfg.sclk.mux  = I2S_MUX;
    i2s_params.pin_cfg.sclk.pin  = I2S_SCLK_PIN;
    i2s_params.pin_cfg.sclk.pull = APP_IO_NOPULL;

    i2s_params.pin_cfg.ws.type = APP_IO_TYPE_NORMAL;
    i2s_params.pin_cfg.ws.mux  = I2S_MUX;
    i2s_params.pin_cfg.ws.pin  = I2S_WS_PIN;
    i2s_params.pin_cfg.ws.pull = APP_IO_NOPULL;

    i2s_params.pin_cfg.sdo.type = APP_IO_TYPE_NORMAL;
    i2s_params.pin_cfg.sdo.mux  = I2S_MUX;
    i2s_params.pin_cfg.sdo.pin  = I2S_TX_PIN;
    i2s_params.pin_cfg.sdo.pull = APP_IO_NOPULL;

    i2s_params.pin_cfg.sdi.type = APP_IO_TYPE_NORMAL;
    i2s_params.pin_cfg.sdi.mux  = I2S_MUX;
    i2s_params.pin_cfg.sdi.pin  = I2S_RX_PIN;
    i2s_params.pin_cfg.sdi.pull = APP_IO_NOPULL;


		i2s_params.init.data_size    = I2S_DATASIZE_16BIT;
    i2s_params.init.clock_source = I2S_CLOCK_SRC_96M; // 蹇呴』浣跨敤 96M 鏃堕挓婧愶紙鎴栬€?2M婧愶級锛?浼氬鑷存椂閽熷け鏁?
    i2s_params.init.audio_freq   = 48000;             // 48kHz
		
    i2s_params.use_mode.type = APP_I2S_TYPE_POLLING;
    i2s_params.use_mode.tx_dma_channel = DMA_Channel0; // 瑙嗗疄闄呭彲鐢ㄩ€氶亾璋冩暣
    i2s_params.use_mode.rx_dma_channel = DMA_Channel1;
   
    
    app_i2s_init(&i2s_params, NULL);
}

void audio_hardware_init(void)
{
    APP_LOG_INFO("Hardware init for INMP441 - MIC & WM8960");
    
    i2c_bus_init();
    if (!wm8960_probe()) {
        APP_LOG_ERROR("WM8960 probe failed!");
    } else {
        wm8960_init();
    }
    
    i2s_bus_init();
    
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

#include "app_log.h"
#include "gus.h" // 寮曞叆钃濈墮鍙戦€佸嚱鏁?

void audio_loopback_process(void)
{
    static uint32_t debug_cnt = 0;
    
    // 濡傛灉 SDK 涓病鏈夌洿鎺ョ殑鍙屽悜 API (app_i2s_transmit_receive_sync)锛屽彲浠ラ€氳繃鍏堝悗璋冪敤鎴栬€?DMA 鍒嗗埆鍚敤鏉ュ疄鐜般€?
    // 鍦ㄨ疆璇㈡ā寮忎笅锛屽彲浠ュ厛鎺ユ敹鍚庡彂閫併€?
    if (app_i2s_receive_sync(APP_I2S_ID_MASTER, (uint16_t*)rx_buffer, AUDIO_FRAME_SAMPLES * 2, 100) == APP_DRV_SUCCESS)
    {
        int in_index = MIC_USE_LEFT_SLOT ? 0 : 1;
        int16_t sample_val = rx_buffer[in_index]; // 鎻愬彇绗竴涓牱鏈?

        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++)
        {
            int16_t s16 = rx_buffer[2 * i + in_index];

            // 绠€鍗?DC blocker
            float x = (float)s16;
            float y = x - dc_block_x + 0.995f * dc_block_y;
            dc_block_x = x;
            dc_block_y = y;

            int16_t out16 = (int16_t)y;

            #if DIAG_RIGHT_ONLY
            tx_buffer[2 * i] = 0;
            tx_buffer[2 * i + 1] = out16;
            #else
            tx_buffer[2 * i] = out16;
            tx_buffer[2 * i + 1] = out16;
            #endif
        }
        
        // 鎺ユ敹鍜屽鐞嗗畬鍚庯紝鍐嶅悓姝ュ彂閫佸嚭鍘?
        // app_i2s_transmit_sync(APP_I2S_ID_MASTER, (uint16_t*)tx_buffer, AUDIO_FRAME_SAMPLES * 2, 100);

        // 姣忛殧涓€娈垫椂闂达紝閫氳繃钃濈墮鎶婂０闊虫暟鎹彂鍒版墜鏈虹锛?
        if (++debug_cnt >= 5) {
            debug_cnt = 0;
            uint8_t ble_tx_data[4];
            ble_tx_data[0] = 0xAA; // 甯уご锛屼唬琛ㄩ噰闊虫垚鍔?
            ble_tx_data[1] = (sample_val >> 8) & 0xFF; // 闊抽楂?浣?
            ble_tx_data[2] = sample_val & 0xFF;        // 闊抽浣?浣?
            ble_tx_data[3] = 0xBB; // 甯у熬
            gus_tx_data_send(0, ble_tx_data, 4); // 閫氳繃钃濈墮鍙戦€?
        }
    } else {
        if (++debug_cnt >= 5) {
            debug_cnt = 0;
            uint8_t ble_err_data[4] = {0xEE, 0xEE, 0xEE, 0xEE}; // EEEE琛ㄧずI2S鎺ユ敹澶辫触
            gus_tx_data_send(0, ble_err_data, 4);
        }
    }
}
