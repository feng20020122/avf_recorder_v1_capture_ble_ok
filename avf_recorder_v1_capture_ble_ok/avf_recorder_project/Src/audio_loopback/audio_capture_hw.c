#include "audio_capture_hw.h"
#include "app_i2c.h"
#include "app_log.h"
#include "gr55xx_delay.h"
#include "audio_storage.h"
#include "transport_scheduler.h"
#include <string.h>

/* I2C 引脚配置，严密遵循 GR5515-AVF PDF 原理图 */
#define I2C_SCL_PIN      APP_IO_PIN_30 // WM_SCL -> IO30
#define I2C_SDA_PIN      APP_IO_PIN_26 // WM_SDA -> IO26

/* I2S_M 引脚配置(Master)，严密遵循 GR5515-AVF PDF 原理图 */
#define I2S_WS_PIN       APP_IO_PIN_24 // I2S_LRCLK/ICS_WS -> IO24
#define I2S_SCLK_PIN     APP_IO_PIN_17 // I2S_CLK/ICS_SCK -> IO17
#define I2S_TX_PIN       APP_IO_PIN_25 // I2S_DAC -> IO25
#define I2S_RX_PIN       APP_IO_PIN_16 // ICS_SD -> IO16
#define I2S_MUX          APP_IO_MUX_3  // IO16,17,24,25 的 I2SM 复用均为 MUX_3

#define WM8962_ENABLE_INTERNAL_BEEP_TEST  1u
#define WM8962_ENABLE_I2S_TONE_TEST       0u

static uint8_t g_wm8962_addr = WM8962_I2C_ADDR_1;

#define WM8962_HPOUTL_VOLUME             0x0002u
#define WM8962_HPOUTR_VOLUME             0x0003u
#define WM8962_ADC_DAC_CONTROL_1         0x0005u
#define WM8962_ADC_DAC_CONTROL_2         0x0006u
#define WM8962_AUDIO_INTERFACE_0         0x0007u
#define WM8962_CLOCKING2                 0x0008u
#define WM8962_AUDIO_INTERFACE_1         0x0009u
#define WM8962_LEFT_DAC_VOLUME           0x000Au
#define WM8962_RIGHT_DAC_VOLUME          0x000Bu
#define WM8962_AUDIO_INTERFACE_2         0x000Eu
#define WM8962_SOFTWARE_RESET            0x000Fu
#define WM8962_ADDITIONAL_CONTROL_1      0x0017u
#define WM8962_PWR_MGMT_1                0x0019u
#define WM8962_PWR_MGMT_2                0x001Au
#define WM8962_ADDITIONAL_CONTROL_3      0x001Bu
#define WM8962_ANTI_POP                  0x001Cu
#define WM8962_CLOCKING_3                0x001Eu
#define WM8962_CLOCKING_4                0x0038u
#define WM8962_DC_SERVO_1                0x003Du
#define WM8962_DC_SERVO_6                0x0042u
#define WM8962_ANALOGUE_HP_0             0x0045u
#define WM8962_ANALOGUE_HP_2             0x0047u
#define WM8962_CHARGE_PUMP_1             0x0048u
#define WM8962_CHARGE_PUMP_B             0x0052u
#define WM8962_MIXER_ENABLES             0x0063u
#define WM8962_HEADPHONE_MIXER_1         0x0064u
#define WM8962_HEADPHONE_MIXER_2         0x0065u
#define WM8962_HEADPHONE_MIXER_3         0x0066u
#define WM8962_HEADPHONE_MIXER_4         0x0067u
#define WM8962_BEEP_GENERATOR_1          0x006Eu
#define WM8962_ANALOGUE_CLOCKING1        0x007Cu
#define WM8962_ANALOGUE_CLOCKING2        0x007Du
#define WM8962_ANALOGUE_CLOCKING3        0x007Eu

#define WM8962_CHIP_ID_VALUE             0x6243u
#define WM8962_CLOCKING2_MCLK_DEFAULT    0x01E4u
#define WM8962_CLOCKING4_DEFAULT         0x0506u
#define WM8962_AUDIO_IF0_DEFAULT         0x000Au
#define WM8962_AUDIO_IF1_DEFAULT         0x0300u
#define WM8962_AUDIO_IF2_DEFAULT         0x0040u
#define WM8962_ADDITIONAL_CTRL1_DEFAULT  0x0160u
#define WM8962_ADC_DAC_CTRL2_DEFAULT     0x2008u
#define WM8962_ANALOGUE_CLOCKING1_DEFAULT 0x0011u
#define WM8962_ANALOGUE_CLOCKING2_DEFAULT 0x004Bu
#define WM8962_ANALOGUE_CLOCKING3_DEFAULT 0x000Du
#define WM8962_CLKREG_OVD                0x0800u
#define WM8962_SYSCLK_ENA                0x0020u
#define WM8962_DAC_MUTE                  0x0008u
#define WM8962_TOCLK_ENA                 0x0001u
#define WM8962_BIAS_ENA                  0x0040u
#define WM8962_VMID_FAST_START           0x0180u
#define WM8962_DACL_ENA                  0x0100u
#define WM8962_DACR_ENA                  0x0080u
#define WM8962_HPOUTL_PGA_ENA            0x0040u
#define WM8962_HPOUTR_PGA_ENA            0x0020u
#define WM8962_HPOUT_MUX_DIRECT_DAC      0x0000u
#define WM8962_HPMIXL_ENA                0x0008u
#define WM8962_HPMIXR_ENA                0x0004u
#define WM8962_HPMIXL_TO_HPOUTL_PGA      0x0080u
#define WM8962_DACL_TO_HPMIXL            0x0020u
#define WM8962_HPMIXR_TO_HPOUTR_PGA      0x0080u
#define WM8962_DACR_TO_HPMIXR            0x0010u
#define WM8962_HPMIX_ENABLES             (WM8962_HPMIXL_ENA | WM8962_HPMIXR_ENA)
#define WM8962_HPOUTL_MIXER_PATH         (WM8962_HPMIXL_TO_HPOUTL_PGA | WM8962_DACL_TO_HPMIXL)
#define WM8962_HPOUTR_MIXER_PATH         (WM8962_HPMIXR_TO_HPOUTR_PGA | WM8962_DACR_TO_HPMIXR)
#define WM8962_HP_ANALOGUE_ENABLE        0x00FFu
#define WM8962_CP_ENA                    0x0001u
#define WM8962_CHARGE_PUMP_B_DEFAULT     0x0004u
#define WM8962_CLOCKING2_BEEP_TEST       (WM8962_CLOCKING2_MCLK_DEFAULT | WM8962_CLKREG_OVD | WM8962_SYSCLK_ENA)
#define WM8962_ADDITIONAL_CTRL1_TOCLK    (WM8962_ADDITIONAL_CTRL1_DEFAULT | WM8962_TOCLK_ENA)
#define WM8962_HPOUT_VOLUME              0x013Fu
#define WM8962_DAC_VOLUME                0x01C0u
#define WM8962_BEEP_1KHZ_MAX_GAIN        0x00F3u

// 32-bit stereo I2S input, converted to mono 16-bit PCM for WAV storage.
static int16_t storage_pcm_buffer[AUDIO_FRAME_SAMPLES];

// 调试变量
volatile int32_t g_debug_mic_amp = 0;
volatile uint32_t g_debug_i2s_rx_fail_count = 0;
volatile uint32_t g_debug_i2s_overrun_count = 0;
volatile uint32_t g_debug_i2s_queue_max = 0;
#if WM8962_ENABLE_I2S_TONE_TEST
volatile uint32_t g_debug_i2s_tx_ok_count = 0;
volatile uint32_t g_debug_i2s_tx_fail_count = 0;
volatile uint8_t g_debug_i2s_tone_active = 0;
#endif
volatile uint16_t g_debug_wm8962_chip_id = 0;
volatile uint16_t g_debug_wm8962_last_reg = 0;
volatile uint16_t g_debug_wm8962_last_write = 0;
volatile uint16_t g_debug_wm8962_last_read = 0;
volatile uint32_t g_debug_wm8962_mismatch_count = 0;
volatile uint32_t g_debug_wm8962_write_fail_count = 0;
volatile uint8_t g_debug_wm8962_beep_ok = 0;
volatile uint16_t g_debug_wm8962_dc_servo_6 = 0;
volatile uint16_t g_debug_wm8962_reg_clocking2 = 0;
volatile uint16_t g_debug_wm8962_reg_clocking4 = 0;
volatile uint16_t g_debug_wm8962_reg_add1 = 0;
volatile uint16_t g_debug_wm8962_reg_pwr1 = 0;
volatile uint16_t g_debug_wm8962_reg_pwr2 = 0;
volatile uint16_t g_debug_wm8962_reg_hp0 = 0;
volatile uint16_t g_debug_wm8962_reg_cp1 = 0;
volatile uint16_t g_debug_wm8962_reg_cpb = 0;
volatile uint16_t g_debug_wm8962_reg_mixer_en = 0;
volatile uint16_t g_debug_wm8962_reg_mix1 = 0;
volatile uint16_t g_debug_wm8962_reg_mix2 = 0;
volatile uint16_t g_debug_wm8962_reg_mix3 = 0;
volatile uint16_t g_debug_wm8962_reg_mix4 = 0;
volatile uint16_t g_debug_wm8962_reg_beep = 0;

static float dc_block_x = 0.0f;
static float dc_block_y = 0.0f;

static app_i2c_id_t g_active_i2c_id = APP_I2C_ID_0;
static bool g_i2s_initialized = false;
static volatile bool g_audio_capture_active = false; 
#if WM8962_ENABLE_I2S_TONE_TEST
static volatile bool g_i2s_tone_active = false;
#endif

// DMA stores raw 32-bit stereo I2S frames into a small ring queue.
#define RX_DMA_BUFFER_COUNT   4u
#define RX_DMA_BUFFER_WORDS   (AUDIO_FRAME_SAMPLES * 2)
static int32_t rx_dma_buffer[RX_DMA_BUFFER_COUNT][RX_DMA_BUFFER_WORDS];

#if WM8962_ENABLE_I2S_TONE_TEST
#define TX_TONE_SAMPLES       256u
#define TX_TONE_WORDS         (TX_TONE_SAMPLES * 2u)
#define TX_TONE_AMP           0x20000000
static int32_t tx_tone_buffer[TX_TONE_WORDS];
#endif

static volatile uint8_t g_rx_dma_write_index = 0;
static volatile uint8_t g_rx_dma_read_index = 0;
static volatile uint8_t g_rx_dma_ready_count = 0;
static bool wm8962_read_reg(uint16_t reg, uint16_t *p_val);
#if WM8962_ENABLE_I2S_TONE_TEST
static uint16_t audio_tone_start(void);
static void audio_tone_stop(void);
#endif
/* -------------------------------------------------------------------------
 * 底层总线初始化 
 * ------------------------------------------------------------------------- */
static void i2c_bus_init(void)
{
    app_i2c_params_t i2c_params;
    memset(&i2c_params, 0, sizeof(i2c_params));

    g_active_i2c_id = APP_I2C_ID_0;
    i2c_params.id = APP_I2C_ID_0;
    i2c_params.role = APP_I2C_ROLE_MASTER;
    i2c_params.pin_cfg.scl.type = APP_IO_TYPE_NORMAL;
    i2c_params.pin_cfg.scl.mux  = APP_IO_MUX_2;
    i2c_params.pin_cfg.scl.pin  = I2C_SCL_PIN;
    i2c_params.pin_cfg.scl.pull = APP_IO_PULLUP;
    i2c_params.pin_cfg.sda.type = APP_IO_TYPE_NORMAL;
    i2c_params.pin_cfg.sda.mux  = APP_IO_MUX_2;
    i2c_params.pin_cfg.sda.pin  = I2C_SDA_PIN;
    i2c_params.pin_cfg.sda.pull = APP_IO_PULLUP;
    i2c_params.use_mode.type = APP_I2C_TYPE_POLLING;
    i2c_params.init.speed = I2C_SPEED_100K;
    i2c_params.init.own_address = 0x55;
    i2c_params.init.addressing_mode = I2C_ADDRESSINGMODE_7BIT;
    i2c_params.init.general_call_mode = I2C_GENERALCALL_DISABLE;

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

    i2s_params.init.data_size    = I2S_DATASIZE_32BIT;
    i2s_params.init.clock_source = I2S_CLOCK_SRC_96M; 
    i2s_params.init.audio_freq   = AUDIO_SAMPLE_RATE;             

    i2s_params.use_mode.type = APP_I2S_TYPE_DMA;
    i2s_params.use_mode.tx_dma_channel = DMA_Channel2; 
    i2s_params.use_mode.rx_dma_channel = DMA_Channel3;

    if (app_i2s_init(&i2s_params, app_i2s_dma_rx_cplt_callback) == APP_DRV_SUCCESS)
    {
        g_i2s_initialized = true;
    }
}

static bool wm8962_write_reg(uint16_t reg, uint16_t val)
{
    uint8_t data[4];
    data[0] = (reg >> 8) & 0xFF; // addr MSB
    data[1] = reg & 0xFF;        // addr LSB
    data[2] = (val >> 8) & 0xFF; // data MSB
    data[3] = val & 0xFF;        // data LSB

    uint16_t ret = app_i2c_transmit_sync(g_active_i2c_id, g_wm8962_addr, data, 4, 100);
    return (ret == APP_DRV_SUCCESS);
}

static bool wm8962_write_reg_checked(uint16_t reg, uint16_t val, const char* name)
{
    uint16_t read_val = 0;
    bool ok = wm8962_write_reg(reg, val);
    (void)name;

    g_debug_wm8962_last_reg = reg;
    g_debug_wm8962_last_write = val;

    if (!ok)
    {
        g_debug_wm8962_write_fail_count++;
        return false;
    }

    if (wm8962_read_reg(reg, &read_val))
    {
        g_debug_wm8962_last_read = read_val;
        if (read_val != val)
        {
            g_debug_wm8962_mismatch_count++;
        }
    }
    else
    {
        g_debug_wm8962_write_fail_count++;
        return false;
    }

    return true;
}

static bool wm8962_read_reg(uint16_t reg, uint16_t *p_val)
{
    uint8_t reg_addr[2];
    uint8_t rx_data[2];

    reg_addr[0] = (uint8_t)(reg >> 8);
    reg_addr[1] = (uint8_t)reg;

    if (app_i2c_transmit_sync(g_active_i2c_id, g_wm8962_addr, reg_addr, sizeof(reg_addr), 100) != APP_DRV_SUCCESS) {
        return false;
    }
    if (app_i2c_receive_sync(g_active_i2c_id, g_wm8962_addr, rx_data, sizeof(rx_data), 100) != APP_DRV_SUCCESS) {
        return false;
    }

    *p_val = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    return true;
}

static void wm8962_debug_read_snapshot(void)
{
    uint16_t val;

    if (wm8962_read_reg(WM8962_CLOCKING2, &val)) g_debug_wm8962_reg_clocking2 = val;
    if (wm8962_read_reg(WM8962_CLOCKING_4, &val)) g_debug_wm8962_reg_clocking4 = val;
    if (wm8962_read_reg(WM8962_ADDITIONAL_CONTROL_1, &val)) g_debug_wm8962_reg_add1 = val;
    if (wm8962_read_reg(WM8962_PWR_MGMT_1, &val)) g_debug_wm8962_reg_pwr1 = val;
    if (wm8962_read_reg(WM8962_PWR_MGMT_2, &val)) g_debug_wm8962_reg_pwr2 = val;
    if (wm8962_read_reg(WM8962_ANALOGUE_HP_0, &val)) g_debug_wm8962_reg_hp0 = val;
    if (wm8962_read_reg(WM8962_CHARGE_PUMP_1, &val)) g_debug_wm8962_reg_cp1 = val;
    if (wm8962_read_reg(WM8962_CHARGE_PUMP_B, &val)) g_debug_wm8962_reg_cpb = val;
    if (wm8962_read_reg(WM8962_MIXER_ENABLES, &val)) g_debug_wm8962_reg_mixer_en = val;
    if (wm8962_read_reg(WM8962_HEADPHONE_MIXER_1, &val)) g_debug_wm8962_reg_mix1 = val;
    if (wm8962_read_reg(WM8962_HEADPHONE_MIXER_2, &val)) g_debug_wm8962_reg_mix2 = val;
    if (wm8962_read_reg(WM8962_HEADPHONE_MIXER_3, &val)) g_debug_wm8962_reg_mix3 = val;
    if (wm8962_read_reg(WM8962_HEADPHONE_MIXER_4, &val)) g_debug_wm8962_reg_mix4 = val;
    if (wm8962_read_reg(WM8962_BEEP_GENERATOR_1, &val)) g_debug_wm8962_reg_beep = val;
}

static bool wm8962_probe(void)
{
    const uint8_t addrs[2] = {WM8962_I2C_ADDR_1, WM8962_I2C_ADDR_2};
    uint16_t chip_id = 0;

    g_active_i2c_id = APP_I2C_ID_0;
    i2c_bus_init();

    for (int i = 0; i < 2; i++) {
        g_wm8962_addr = addrs[i];
        if (wm8962_read_reg(0x000F, &chip_id)) {
            g_debug_wm8962_chip_id = chip_id;
            if (chip_id == WM8962_CHIP_ID_VALUE) {
                return true;
            }
        }
    }

    return false;
}
static bool wm8962_init(void)
{
    uint16_t dc_servo = 0;

    g_debug_wm8962_mismatch_count = 0;
    g_debug_wm8962_write_fail_count = 0;
    g_debug_wm8962_beep_ok = 0;
    g_debug_wm8962_dc_servo_6 = 0;
    g_debug_wm8962_reg_clocking2 = 0;
    g_debug_wm8962_reg_clocking4 = 0;
    g_debug_wm8962_reg_add1 = 0;
    g_debug_wm8962_reg_pwr1 = 0;
    g_debug_wm8962_reg_pwr2 = 0;
    g_debug_wm8962_reg_hp0 = 0;
    g_debug_wm8962_reg_cp1 = 0;
    g_debug_wm8962_reg_cpb = 0;
    g_debug_wm8962_reg_mixer_en = 0;
    g_debug_wm8962_reg_mix1 = 0;
    g_debug_wm8962_reg_mix2 = 0;
    g_debug_wm8962_reg_mix3 = 0;
    g_debug_wm8962_reg_mix4 = 0;
    g_debug_wm8962_reg_beep = 0;

    (void)wm8962_write_reg(WM8962_SOFTWARE_RESET, 0x0000);
    delay_ms(20);

    if (!wm8962_write_reg_checked(WM8962_ADC_DAC_CONTROL_1, 0x0000, "ADC_DAC1")) return false;
    if (!wm8962_write_reg_checked(WM8962_ADC_DAC_CONTROL_2, WM8962_ADC_DAC_CTRL2_DEFAULT, "ADC_DAC2")) return false;
    if (!wm8962_write_reg_checked(WM8962_AUDIO_INTERFACE_0, WM8962_AUDIO_IF0_DEFAULT, "AIF0")) return false;
    if (!wm8962_write_reg_checked(WM8962_AUDIO_INTERFACE_1, WM8962_AUDIO_IF1_DEFAULT, "AIF1")) return false;
    if (!wm8962_write_reg_checked(WM8962_AUDIO_INTERFACE_2, WM8962_AUDIO_IF2_DEFAULT, "AIF2")) return false;
    if (!wm8962_write_reg_checked(WM8962_CLOCKING_4, WM8962_CLOCKING4_DEFAULT, "CLK4")) return false;
    if (!wm8962_write_reg_checked(WM8962_ADDITIONAL_CONTROL_3, 0x0000, "ADD3")) return false;
    if (!wm8962_write_reg_checked(WM8962_ANALOGUE_CLOCKING1, WM8962_ANALOGUE_CLOCKING1_DEFAULT, "ACLK1")) return false;
    if (!wm8962_write_reg_checked(WM8962_ANALOGUE_CLOCKING2, WM8962_ANALOGUE_CLOCKING2_DEFAULT, "ACLK2")) return false;
    if (!wm8962_write_reg_checked(WM8962_ANALOGUE_CLOCKING3, WM8962_ANALOGUE_CLOCKING3_DEFAULT, "ACLK3")) return false;

    if (!wm8962_write_reg_checked(WM8962_CLOCKING2, WM8962_CLOCKING2_BEEP_TEST, "CLOCKING2")) return false;
    delay_ms(2);
    if (!wm8962_write_reg_checked(WM8962_ADDITIONAL_CONTROL_1, WM8962_ADDITIONAL_CTRL1_TOCLK, "TOCLK")) return false;
    if (!wm8962_write_reg_checked(WM8962_PWR_MGMT_1, WM8962_BIAS_ENA | WM8962_VMID_FAST_START, "PWR1")) return false;
    delay_ms(100);

    if (!wm8962_write_reg_checked(WM8962_PWR_MGMT_2,
                                  WM8962_DACL_ENA | WM8962_DACR_ENA |
                                  WM8962_HPOUTL_PGA_ENA | WM8962_HPOUTR_PGA_ENA,
                                  "PWR2")) return false;

    if (!wm8962_write_reg_checked(WM8962_ADC_DAC_CONTROL_1, 0x0000, "DAC_UNMUTE")) return false;
    if (!wm8962_write_reg_checked(WM8962_LEFT_DAC_VOLUME, WM8962_DAC_VOLUME, "LDAC_VOL")) return false;
    if (!wm8962_write_reg_checked(WM8962_RIGHT_DAC_VOLUME, WM8962_DAC_VOLUME, "RDAC_VOL")) return false;
    if (!wm8962_write_reg_checked(WM8962_HPOUTL_VOLUME, WM8962_HPOUT_VOLUME, "HPOUTL_VOL")) return false;
    if (!wm8962_write_reg_checked(WM8962_HPOUTR_VOLUME, WM8962_HPOUT_VOLUME, "HPOUTR_VOL")) return false;

    if (!wm8962_write_reg_checked(WM8962_MIXER_ENABLES, WM8962_HPMIX_ENABLES, "MIXER_EN")) return false;
    if (!wm8962_write_reg_checked(WM8962_HEADPHONE_MIXER_1,
                                  WM8962_HPOUTL_MIXER_PATH,
                                  "HPOUTL_MIX")) return false;
    if (!wm8962_write_reg_checked(WM8962_HEADPHONE_MIXER_2,
                                  WM8962_HPOUTR_MIXER_PATH,
                                  "HPOUTR_MIX")) return false;
    if (!wm8962_write_reg_checked(WM8962_HEADPHONE_MIXER_3, 0x0000, "HPMIXL_VOL")) return false;
    if (!wm8962_write_reg_checked(WM8962_HEADPHONE_MIXER_4, 0x0000, "HPMIXR_VOL")) return false;

    if (!wm8962_write_reg_checked(WM8962_CHARGE_PUMP_B, WM8962_CHARGE_PUMP_B_DEFAULT, "CP_B")) return false;
    if (!wm8962_write_reg_checked(WM8962_CHARGE_PUMP_1, WM8962_CP_ENA, "CP1")) return false;
    delay_ms(10);

    if (!wm8962_write_reg_checked(WM8962_ANALOGUE_HP_2, 0x01FF, "ANA_HP2")) return false;
    if (!wm8962_write_reg_checked(WM8962_ANALOGUE_HP_0, WM8962_HP_ANALOGUE_ENABLE, "ANA_HP0")) return false;

    (void)wm8962_write_reg_checked(WM8962_DC_SERVO_1, 0x00CC, "DC_SERVO1");
    delay_ms(20);
    if (wm8962_read_reg(WM8962_DC_SERVO_6, &dc_servo))
    {
        g_debug_wm8962_dc_servo_6 = dc_servo;
    }

#if WM8962_ENABLE_INTERNAL_BEEP_TEST
    if (!wm8962_write_reg_checked(WM8962_BEEP_GENERATOR_1, WM8962_BEEP_1KHZ_MAX_GAIN, "BEEP")) return false;
#else
    if (!wm8962_write_reg_checked(WM8962_BEEP_GENERATOR_1, 0x0000, "BEEP_OFF")) return false;
#endif

    wm8962_debug_read_snapshot();
    g_debug_wm8962_beep_ok = (g_debug_wm8962_write_fail_count == 0u) ? 1u : 0u;
    return true;
}

void audio_hardware_init(void)
{
    APP_LOG_INFO("=== Audio Hardware Init (WM8962) ===");

    if (wm8962_probe()) {
        APP_LOG_INFO("WM8962 found at I2C address 0x%02X", g_wm8962_addr);
        if (wm8962_init()) {
            APP_LOG_INFO("WM8962 initialized successfully!");
#if WM8962_ENABLE_I2S_TONE_TEST
            (void)audio_tone_start();
#endif
        } else {
            APP_LOG_ERROR("WM8962 initialization failed.");
        }
    } else {
        APP_LOG_ERROR("WM8962 not found or ID mismatch.");
    }

}

static void audio_capture_queue_reset(void)
{
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));
    g_rx_dma_write_index = 0;
    g_rx_dma_read_index = 0;
    g_rx_dma_ready_count = 0;
    g_debug_i2s_queue_max = 0;
    g_debug_i2s_overrun_count = 0;
}

uint16_t audio_capture_start(void)
{
    uint16_t ret;

    if (g_audio_capture_active)
    {
        return APP_DRV_SUCCESS;
    }

#if WM8962_ENABLE_I2S_TONE_TEST
    audio_tone_stop();
#endif

    if (!g_i2s_initialized)
    {
        i2s_bus_init();
        if (!g_i2s_initialized)
        {
            g_debug_i2s_rx_fail_count++;
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    audio_capture_queue_reset();
    ret = app_i2s_receive_async(APP_I2S_ID_MASTER, (uint16_t *)rx_dma_buffer[0], RX_DMA_BUFFER_WORDS);
    if (ret == APP_DRV_SUCCESS)
    {
        g_audio_capture_active = true;
    }
    else
    {
        g_debug_i2s_rx_fail_count++;
    }

    return ret;
}

void audio_capture_stop(void)
{
    if (g_i2s_initialized)
    {
        g_audio_capture_active = false;
        (void)app_i2s_deinit(APP_I2S_ID_MASTER);
        g_i2s_initialized = false;
        memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));
        g_rx_dma_write_index = 0;
        g_rx_dma_read_index = 0;
        g_rx_dma_ready_count = 0;
    }
}

bool audio_capture_is_active(void)
{
    return g_audio_capture_active;
}

#if WM8962_ENABLE_I2S_TONE_TEST
static void audio_tone_fill_buffer(void)
{
    for (uint32_t i = 0; i < TX_TONE_SAMPLES; i++)
    {
        int32_t sample = ((i & 0x08u) == 0u) ? TX_TONE_AMP : -TX_TONE_AMP;
        tx_tone_buffer[2u * i] = sample;
        tx_tone_buffer[2u * i + 1u] = sample;
    }
}

static uint16_t audio_tone_start(void)
{
    uint16_t ret;

    if (g_i2s_tone_active)
    {
        return APP_DRV_SUCCESS;
    }

    if (!g_i2s_initialized)
    {
        i2s_bus_init();
        if (!g_i2s_initialized)
        {
            g_debug_i2s_tx_fail_count++;
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    audio_tone_fill_buffer();
    ret = app_i2s_transmit_async(APP_I2S_ID_MASTER, (uint16_t *)tx_tone_buffer, TX_TONE_WORDS);
    if (ret == APP_DRV_SUCCESS)
    {
        g_i2s_tone_active = true;
        g_debug_i2s_tone_active = 1;
    }
    else
    {
        g_debug_i2s_tx_fail_count++;
    }

    return ret;
}

static void audio_tone_stop(void)
{
    if (g_i2s_tone_active && g_i2s_initialized)
    {
        g_i2s_tone_active = false;
        g_debug_i2s_tone_active = 0;
        (void)app_i2s_deinit(APP_I2S_ID_MASTER);
        g_i2s_initialized = false;
    }
}
#endif

// DMA receive-complete callback. Keep it short: queue the filled buffer and arm the next free one.
void app_i2s_dma_rx_cplt_callback(app_i2s_evt_t *p_evt)
{
    uint8_t next_write_index;
    uint16_t ret;

    if (p_evt == NULL)
    {
        return;
    }

#if WM8962_ENABLE_I2S_TONE_TEST
    if (p_evt->type == APP_I2S_EVT_TX_CPLT)
    {
        if (g_i2s_tone_active)
        {
            ret = app_i2s_transmit_async(APP_I2S_ID_MASTER, (uint16_t *)tx_tone_buffer, TX_TONE_WORDS);
            if (ret == APP_DRV_SUCCESS)
            {
                g_debug_i2s_tx_ok_count++;
            }
            else
            {
                g_debug_i2s_tx_fail_count++;
            }
        }
        return;
    }
#endif

    if (!g_audio_capture_active)
    {
        return;
    }

    if (p_evt->type == APP_I2S_EVT_RX_DATA)
    {
        next_write_index = (uint8_t)((g_rx_dma_write_index + 1u) % RX_DMA_BUFFER_COUNT);

        if (g_rx_dma_ready_count < RX_DMA_BUFFER_COUNT)
        {
            g_rx_dma_ready_count++;
        }

        if (g_rx_dma_ready_count >= RX_DMA_BUFFER_COUNT)
        {
            g_debug_i2s_overrun_count++;
            g_rx_dma_read_index = (uint8_t)((g_rx_dma_read_index + 1u) % RX_DMA_BUFFER_COUNT);
            g_rx_dma_ready_count = RX_DMA_BUFFER_COUNT - 1u;
        }

        if (g_rx_dma_ready_count > g_debug_i2s_queue_max)
        {
            g_debug_i2s_queue_max = g_rx_dma_ready_count;
        }

        g_rx_dma_write_index = next_write_index;
        ret = app_i2s_receive_async(APP_I2S_ID_MASTER,
                                    (uint16_t *)rx_dma_buffer[g_rx_dma_write_index],
                                    RX_DMA_BUFFER_WORDS);
        if (ret != APP_DRV_SUCCESS)
        {
            g_debug_i2s_rx_fail_count++;
        }
    }
    else if (p_evt->type == APP_I2S_EVT_ERROR)
    {
        g_debug_i2s_rx_fail_count++;
    }
}

static void audio_process_rx_buffer(int32_t *process_buffer)
{
    int32_t max_amp = 0;
    int32_t max_left = 0;
    int32_t max_right = 0;
    int in_index;

    for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++)
    {
        int16_t left = (int16_t)(process_buffer[2 * i] >> 16);
        int16_t right = (int16_t)(process_buffer[2 * i + 1] >> 16);

        if (left > max_left) max_left = left;
        if (-left > max_left) max_left = -left;
        if (right > max_right) max_right = right;
        if (-right > max_right) max_right = -right;
    }

    in_index = (max_right > max_left) ? 1 : 0;

    for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++)
    {
        int32_t s32 = process_buffer[2 * i + in_index];
        int16_t s16 = (int16_t)(s32 >> 16);

        if (s16 > max_amp) max_amp = s16;
        if (-s16 > max_amp) max_amp = -s16;

        float x = (float)s16;
        float y = x - dc_block_x + 0.995f * dc_block_y;
        dc_block_x = x;
        dc_block_y = y;
        storage_pcm_buffer[i] = (int16_t)y;
    }

    g_debug_mic_amp = max_amp;
    (void)audio_storage_write_pcm16(storage_pcm_buffer, AUDIO_FRAME_SAMPLES);

    // 实时BLE流送——原始PCM数据，不经任何编码
    // flutter_blue_plus 的腐败Bug由Android原声EventChannel绕过
    if (transport_flag_cfm(GUS_TX_NTF_ENABLE) &&
        ble_ring_has_space(AUDIO_FRAME_SAMPLES * sizeof(int16_t)))
    {
        uart_to_ble_push((const uint8_t *)storage_pcm_buffer,
                         AUDIO_FRAME_SAMPLES * sizeof(int16_t));
        transport_schedule();
    }
}

void audio_capture_process(void)
{
    int32_t *process_buffer;
    uint8_t process_index;

    do
    {
        process_buffer = NULL;

        GLOBAL_EXCEPTION_DISABLE();
        if (g_rx_dma_ready_count > 0u)
        {
            process_index = g_rx_dma_read_index;
            g_rx_dma_read_index = (uint8_t)((g_rx_dma_read_index + 1u) % RX_DMA_BUFFER_COUNT);
            g_rx_dma_ready_count--;
            process_buffer = rx_dma_buffer[process_index];
        }
        GLOBAL_EXCEPTION_ENABLE();

        if (process_buffer != NULL)
        {
            audio_process_rx_buffer(process_buffer);
        }
    } while (process_buffer != NULL);
}
