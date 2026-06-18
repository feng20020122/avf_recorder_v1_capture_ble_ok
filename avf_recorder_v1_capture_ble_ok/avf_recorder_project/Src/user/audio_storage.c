#include "audio_storage.h"
#include "audio_file_transfer.h"

#include <stdio.h>
#include <string.h>

#include "app_io.h"
#include "app_log.h"
#include "app_spi.h"
#include "diskio.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "FreeRTOS.h"
#include "gr55xx_delay.h"
#include "task.h"
#include "transport_scheduler.h"
#include "gus.h"
#include "user_periph_setup.h"

#define TF_CS_PIN                   APP_IO_PIN_15
#define TF_SCK_PIN                  APP_IO_PIN_12
#define TF_MOSI_PIN                 APP_IO_PIN_13
#define TF_MISO_PIN                 APP_IO_PIN_14
#define TF_SPI_MUX                  APP_IO_MUX_1

#define TF_SPI_INIT_HZ              400000u
#define TF_SPI_RUN_HZ               8000000u
#define TF_SECTOR_SIZE              512u
#define TF_SYNC_THRESHOLD_BYTES     (128u * 1024u)
#define WAV_HEADER_SIZE             44u
#define WAV_DEFAULT_SAMPLE_RATE     48000u
#define WAV_DEFAULT_BITS_PER_SAMPLE 16u
#define WAV_DEFAULT_CHANNELS        1u
#define PCM_RING_SAMPLES            32768u
#define PCM_WRITE_CHUNK_SAMPLES     256u

#define CMD0                        (0u)
#define CMD1                        (1u)
#define CMD8                        (8u)
#define CMD9                        (9u)
#define CMD12                       (12u)
#define CMD16                       (16u)
#define CMD17                       (17u)
#define CMD24                       (24u)
#define CMD55                       (55u)
#define CMD58                       (58u)
#define ACMD41                      (0x80u + 41u)

#define CT_MMC                      0x01u
#define CT_SD1                      0x02u
#define CT_SD2                      0x04u
#define CT_SDC                      (CT_SD1 | CT_SD2)
#define CT_BLOCK                    0x08u

static DSTATUS tf_disk_initialize(BYTE lun);
static DSTATUS tf_disk_status(BYTE lun);
static DRESULT tf_disk_read(BYTE lun, BYTE *p_buff, DWORD sector, UINT count);
static DRESULT tf_disk_write(BYTE lun, const BYTE *p_buff, DWORD sector, UINT count);
static DRESULT tf_disk_ioctl(BYTE lun, BYTE cmd, void *p_buff);

static diskio_drv_t s_tf_disk_driver =
{
    tf_disk_initialize,
    tf_disk_status,
    tf_disk_read,
    tf_disk_write,
    tf_disk_ioctl,
};

static FATFS s_fs;
static FIL s_file;
static char s_drive_path[4];
static char s_file_path[24];
static bool s_driver_linked;
static bool s_mounted;
static bool s_recording;
static bool s_ble_send_done;
static uint32_t s_bytes_since_sync;
static uint32_t s_wav_data_bytes;
static uint32_t s_wav_sample_rate = WAV_DEFAULT_SAMPLE_RATE;
static uint16_t s_wav_bits_per_sample = WAV_DEFAULT_BITS_PER_SAMPLE;
static uint16_t s_wav_channels = WAV_DEFAULT_CHANNELS;
static uint32_t s_target_samples;
static uint32_t s_written_samples;
static uint32_t s_captured_samples;
static bool s_capture_done;
static int16_t s_pcm_ring[PCM_RING_SAMPLES];
static int16_t s_pcm_write_chunk[PCM_WRITE_CHUNK_SAMPLES];
static volatile uint32_t s_pcm_ring_read;
static volatile uint32_t s_pcm_ring_write;
static volatile uint32_t s_pcm_ring_count;

volatile uint8_t g_debug_tf_recording;
volatile uint8_t g_debug_tf_last_status;
volatile uint32_t g_debug_tf_written_bytes;
volatile uint8_t g_debug_tf_sync_fresult;
volatile uint8_t g_debug_tf_close_fresult;
volatile uint32_t g_debug_tf_file_size;
volatile uint8_t g_debug_tf_expand_fresult;
volatile uint32_t g_debug_pcm_ring_max;
volatile uint32_t g_debug_pcm_ring_overflow_count;

static DSTATUS s_disk_status = STA_NOINIT;
static uint8_t s_card_type;
static DWORD s_sector_count;
static uint8_t s_ff_tx_sector[TF_SECTOR_SIZE];

static void tf_cs_high(void)
{
    app_io_write_pin(APP_IO_TYPE_NORMAL, TF_CS_PIN, APP_IO_PIN_SET);
}

static void tf_cs_low(void)
{
    app_io_write_pin(APP_IO_TYPE_NORMAL, TF_CS_PIN, APP_IO_PIN_RESET);
}

static uint8_t tf_spi_txrx(uint8_t data)
{
    uint8_t rx = 0xff;

    if (APP_DRV_SUCCESS != app_spi_transmit_receive_sync(APP_SPI_ID_MASTER, &data, &rx, 1, 1000))
    {
        return 0xff;
    }

    return rx;
}

static bool tf_spi_set_speed(uint32_t hz)
{
    app_spi_params_t spi_params;
    uint32_t prescaler;

    memset(&spi_params, 0, sizeof(spi_params));
    (void)app_spi_deinit(APP_SPI_ID_MASTER);

    prescaler = SystemCoreClock / hz;
    if (prescaler < 2u)
    {
        prescaler = 2u;
    }

    spi_params.id = APP_SPI_ID_MASTER;
    spi_params.pin_cfg.cs.enable = APP_SPI_PIN_DISABLE;
    spi_params.pin_cfg.clk.type = APP_IO_TYPE_NORMAL;
    spi_params.pin_cfg.clk.mux = TF_SPI_MUX;
    spi_params.pin_cfg.clk.pin = TF_SCK_PIN;
    spi_params.pin_cfg.clk.pull = APP_IO_NOPULL;
    spi_params.pin_cfg.clk.enable = APP_SPI_PIN_ENABLE;
    spi_params.pin_cfg.mosi.type = APP_IO_TYPE_NORMAL;
    spi_params.pin_cfg.mosi.mux = TF_SPI_MUX;
    spi_params.pin_cfg.mosi.pin = TF_MOSI_PIN;
    spi_params.pin_cfg.mosi.pull = APP_IO_NOPULL;
    spi_params.pin_cfg.mosi.enable = APP_SPI_PIN_ENABLE;
    spi_params.pin_cfg.miso.type = APP_IO_TYPE_NORMAL;
    spi_params.pin_cfg.miso.mux = TF_SPI_MUX;
    spi_params.pin_cfg.miso.pin = TF_MISO_PIN;
    spi_params.pin_cfg.miso.pull = APP_IO_PULLUP;
    spi_params.pin_cfg.miso.enable = APP_SPI_PIN_ENABLE;
    spi_params.use_mode.type = APP_SPI_TYPE_POLLING;
    spi_params.init.data_size = SPI_DATASIZE_8BIT;
    spi_params.init.clock_polarity = SPI_POLARITY_LOW;
    spi_params.init.clock_phase = SPI_PHASE_1EDGE;
    spi_params.init.baudrate_prescaler = prescaler;
    spi_params.init.ti_mode = SPI_TIMODE_DISABLE;
    spi_params.init.slave_select = SPI_SLAVE_SELECT_0;

    return (APP_DRV_SUCCESS == app_spi_init(&spi_params, NULL));
}

static bool tf_spi_init(void)
{
    app_io_init_t cs_cfg = APP_IO_DEFAULT_CONFIG;

    cs_cfg.pin = TF_CS_PIN;
    cs_cfg.mode = APP_IO_MODE_OUT_PUT;
    cs_cfg.pull = APP_IO_PULLUP;
    cs_cfg.mux = APP_IO_MUX_7;
    app_io_init(APP_IO_TYPE_NORMAL, &cs_cfg);
    tf_cs_high();

    return tf_spi_set_speed(TF_SPI_INIT_HZ);
}

static bool tf_wait_ready(uint32_t timeout_ms)
{
    uint8_t resp;

    do
    {
        resp = tf_spi_txrx(0xff);
        if (resp == 0xff)
        {
            return true;
        }
        delay_ms(1);
    } while (timeout_ms--);

    return false;
}

static void tf_deselect(void)
{
    tf_cs_high();
    (void)tf_spi_txrx(0xff);
}

static bool tf_select(void)
{
    tf_cs_low();
    (void)tf_spi_txrx(0xff);

    if (tf_wait_ready(500))
    {
        return true;
    }

    tf_deselect();
    return false;
}

static uint8_t tf_send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t packet[6];
    uint8_t resp;
    uint8_t retry;

    if (cmd & 0x80u)
    {
        cmd &= 0x7fu;
        resp = tf_send_cmd(CMD55, 0);
        if (resp > 1u)
        {
            return resp;
        }
    }

    tf_deselect();
    if (!tf_select())
    {
        return 0xff;
    }

    packet[0] = 0x40u | cmd;
    packet[1] = (uint8_t)(arg >> 24);
    packet[2] = (uint8_t)(arg >> 16);
    packet[3] = (uint8_t)(arg >> 8);
    packet[4] = (uint8_t)arg;
    packet[5] = 0x01u;

    if (cmd == CMD0)
    {
        packet[5] = 0x95u;
    }
    else if (cmd == CMD8)
    {
        packet[5] = 0x87u;
    }

    app_spi_transmit_sync(APP_SPI_ID_MASTER, packet, sizeof(packet), 1000);

    if (cmd == CMD12)
    {
        (void)tf_spi_txrx(0xff);
    }

    retry = 10u;
    do
    {
        resp = tf_spi_txrx(0xff);
    } while ((resp & 0x80u) && --retry);

    return resp;
}

static bool tf_receive_data(uint8_t *p_buff, uint32_t length)
{
    uint8_t token;
    uint32_t wait;

    for (wait = 0; wait < 200u; wait++)
    {
        token = tf_spi_txrx(0xff);
        if (token == 0xfeu)
        {
            if (length == TF_SECTOR_SIZE)
            {
                app_spi_transmit_receive_sync(APP_SPI_ID_MASTER, s_ff_tx_sector, p_buff, TF_SECTOR_SIZE, 5000);
            }
            else
            {
                while (length--)
                {
                    *p_buff++ = tf_spi_txrx(0xff);
                }
            }

            (void)tf_spi_txrx(0xff);
            (void)tf_spi_txrx(0xff);
            return true;
        }
        delay_ms(1);
    }

    return false;
}

static bool tf_transmit_data(const uint8_t *p_buff, uint8_t token)
{
    uint8_t resp;

    if (!tf_wait_ready(500))
    {
        return false;
    }

    (void)tf_spi_txrx(token);
    if (token != 0xfdu)
    {
        app_spi_transmit_sync(APP_SPI_ID_MASTER, (uint8_t *)p_buff, TF_SECTOR_SIZE, 5000);
        (void)tf_spi_txrx(0xff);
        (void)tf_spi_txrx(0xff);

        resp = tf_spi_txrx(0xff);
        if ((resp & 0x1fu) != 0x05u)
        {
            return false;
        }

        if (!tf_wait_ready(1000))
        {
            return false;
        }
    }

    return true;
}

static bool tf_read_csd(uint8_t csd[16])
{
    bool ok = false;

    if (tf_send_cmd(CMD9, 0) == 0)
    {
        ok = tf_receive_data(csd, 16);
    }
    tf_deselect();

    return ok;
}

static DWORD tf_sector_count_from_csd(const uint8_t csd[16])
{
    DWORD csize;
    uint8_t n;

    if ((csd[0] >> 6) == 1u)
    {
        csize = ((DWORD)(csd[7] & 0x3fu) << 16) | ((DWORD)csd[8] << 8) | csd[9];
        return (csize + 1u) << 10;
    }

    n = (uint8_t)((csd[5] & 0x0fu) + ((csd[10] & 0x80u) >> 7) + ((csd[9] & 0x03u) << 1) + 2u);
    csize = ((DWORD)(csd[8] & 0xc0u) >> 6) | ((DWORD)csd[7] << 2) | ((DWORD)(csd[6] & 0x03u) << 10);
    return (csize + 1u) << (n - 9u);
}

static bool tf_card_init(void)
{
    uint8_t i;
    uint8_t resp;
    uint8_t ocr[4];
    uint8_t csd[16];
    uint32_t retry;

    s_card_type = 0;
    s_sector_count = 0;
    memset(s_ff_tx_sector, 0xff, sizeof(s_ff_tx_sector));

    if (!tf_spi_init())
    {
        return false;
    }

    for (i = 0; i < 10u; i++)
    {
        (void)tf_spi_txrx(0xff);
    }

    if (tf_send_cmd(CMD0, 0) != 1u)
    {
        tf_deselect();
        return false;
    }

    resp = tf_send_cmd(CMD8, 0x1aau);
    if (resp == 1u)
    {
        for (i = 0; i < 4u; i++)
        {
            ocr[i] = tf_spi_txrx(0xff);
        }
        if (ocr[2] != 0x01u || ocr[3] != 0xaau)
        {
            tf_deselect();
            return false;
        }

        for (retry = 0; retry < 1000u; retry++)
        {
            if (tf_send_cmd(ACMD41, 0x40000000u) == 0u)
            {
                break;
            }
            delay_ms(1);
        }

        if (retry >= 1000u || tf_send_cmd(CMD58, 0) != 0u)
        {
            tf_deselect();
            return false;
        }

        for (i = 0; i < 4u; i++)
        {
            ocr[i] = tf_spi_txrx(0xff);
        }
        s_card_type = (ocr[0] & 0x40u) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
    }
    else
    {
        if (tf_send_cmd(ACMD41, 0) <= 1u)
        {
            s_card_type = CT_SD1;
            for (retry = 0; retry < 1000u; retry++)
            {
                if (tf_send_cmd(ACMD41, 0) == 0u)
                {
                    break;
                }
                delay_ms(1);
            }
        }
        else
        {
            s_card_type = CT_MMC;
            for (retry = 0; retry < 1000u; retry++)
            {
                if (tf_send_cmd(CMD1, 0) == 0u)
                {
                    break;
                }
                delay_ms(1);
            }
        }

        if (retry >= 1000u || tf_send_cmd(CMD16, TF_SECTOR_SIZE) != 0u)
        {
            s_card_type = 0;
        }
    }

    tf_deselect();
    if (s_card_type == 0)
    {
        return false;
    }

    (void)tf_spi_set_speed(TF_SPI_RUN_HZ);

    if (tf_read_csd(csd))
    {
        s_sector_count = tf_sector_count_from_csd(csd);
    }

    return true;
}

static bool tf_read_sector(uint8_t *p_buff, DWORD sector)
{
    bool ok = false;

    if (!(s_card_type & CT_BLOCK))
    {
        sector *= TF_SECTOR_SIZE;
    }

    if (tf_send_cmd(CMD17, sector) == 0u)
    {
        ok = tf_receive_data(p_buff, TF_SECTOR_SIZE);
    }
    tf_deselect();

    return ok;
}

static bool tf_write_sector(const uint8_t *p_buff, DWORD sector)
{
    bool ok = false;

    if (!(s_card_type & CT_BLOCK))
    {
        sector *= TF_SECTOR_SIZE;
    }

    if (tf_send_cmd(CMD24, sector) == 0u)
    {
        ok = tf_transmit_data(p_buff, 0xfeu);
    }
    tf_deselect();

    return ok;
}

static DSTATUS tf_disk_initialize(BYTE lun)
{
    (void)lun;

    if (tf_card_init())
    {
        s_disk_status = 0;
    }
    else
    {
        s_disk_status = STA_NOINIT;
    }

    return s_disk_status;
}

static DSTATUS tf_disk_status(BYTE lun)
{
    (void)lun;
    return s_disk_status;
}

static DRESULT tf_disk_read(BYTE lun, BYTE *p_buff, DWORD sector, UINT count)
{
    (void)lun;

    if (s_disk_status & STA_NOINIT)
    {
        return RES_NOTRDY;
    }
    if (p_buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }

    while (count--)
    {
        if (!tf_read_sector(p_buff, sector++))
        {
            return RES_ERROR;
        }
        p_buff += TF_SECTOR_SIZE;
    }

    return RES_OK;
}

static DRESULT tf_disk_write(BYTE lun, const BYTE *p_buff, DWORD sector, UINT count)
{
    (void)lun;

    if (s_disk_status & STA_NOINIT)
    {
        return RES_NOTRDY;
    }
    if (p_buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }

    while (count--)
    {
        if (!tf_write_sector(p_buff, sector++))
        {
            return RES_ERROR;
        }
        p_buff += TF_SECTOR_SIZE;
    }

    return RES_OK;
}

static DRESULT tf_disk_ioctl(BYTE lun, BYTE cmd, void *p_buff)
{
    DRESULT res;

    (void)lun;

    if (s_disk_status & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
        case CTRL_SYNC:
            if (!tf_select())
            {
                return RES_ERROR;
            }
            res = tf_wait_ready(1000) ? RES_OK : RES_ERROR;
            tf_deselect();
            return res;

        case GET_SECTOR_COUNT:
            if (p_buff == NULL)
            {
                return RES_PARERR;
            }
            *(DWORD *)p_buff = s_sector_count;
            return RES_OK;

        case GET_SECTOR_SIZE:
            if (p_buff == NULL)
            {
                return RES_PARERR;
            }
            *(WORD *)p_buff = TF_SECTOR_SIZE;
            return RES_OK;

        case GET_BLOCK_SIZE:
            if (p_buff == NULL)
            {
                return RES_PARERR;
            }
            *(DWORD *)p_buff = 1;
            return RES_OK;

        case MMC_GET_TYPE:
            if (p_buff == NULL)
            {
                return RES_PARERR;
            }
            *(BYTE *)p_buff = s_card_type;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}

static void build_file_path(char *p_dst, const char *p_name)
{
    strcpy(p_dst, s_drive_path);
    strncat(p_dst, p_name, sizeof(s_file_path) - strlen(p_dst) - 1u);
}

static void put_u16_le(uint8_t *p_dst, uint16_t value)
{
    p_dst[0] = (uint8_t)value;
    p_dst[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *p_dst, uint32_t value)
{
    p_dst[0] = (uint8_t)value;
    p_dst[1] = (uint8_t)(value >> 8);
    p_dst[2] = (uint8_t)(value >> 16);
    p_dst[3] = (uint8_t)(value >> 24);
}

static FRESULT wav_header_update(void)
{
    uint8_t header[WAV_HEADER_SIZE];
    UINT written;
    DWORD current_pos;
    uint16_t block_align;
    uint32_t byte_rate;
    FRESULT fs_ret;

    memset(header, 0, sizeof(header));
    memcpy(&header[0], "RIFF", 4);
    put_u32_le(&header[4], 36u + s_wav_data_bytes);
    memcpy(&header[8], "WAVE", 4);
    memcpy(&header[12], "fmt ", 4);
    put_u32_le(&header[16], 16u);
    put_u16_le(&header[20], 1u);
    put_u16_le(&header[22], s_wav_channels);
    put_u32_le(&header[24], s_wav_sample_rate);
    block_align = (uint16_t)((s_wav_channels * s_wav_bits_per_sample) / 8u);
    byte_rate = s_wav_sample_rate * block_align;
    put_u32_le(&header[28], byte_rate);
    put_u16_le(&header[32], block_align);
    put_u16_le(&header[34], s_wav_bits_per_sample);
    memcpy(&header[36], "data", 4);
    put_u32_le(&header[40], s_wav_data_bytes);

    current_pos = f_tell(&s_file);
    if (current_pos < WAV_HEADER_SIZE)
    {
        current_pos = WAV_HEADER_SIZE;
    }
    fs_ret = f_lseek(&s_file, 0);
    if (fs_ret != FR_OK)
    {
        return fs_ret;
    }

    fs_ret = f_write(&s_file, header, sizeof(header), &written);
    if (fs_ret != FR_OK || written != sizeof(header))
    {
        return FR_DISK_ERR;
    }

    return f_lseek(&s_file, current_pos);
}

static FRESULT wav_file_preallocate(uint32_t max_samples)
{
    FRESULT fs_ret;
    FSIZE_t target_size;

    if (max_samples == 0u)
    {
        return FR_OK;
    }

    target_size = (FSIZE_t)WAV_HEADER_SIZE +
                  (FSIZE_t)max_samples * (FSIZE_t)s_wav_channels *
                  ((FSIZE_t)s_wav_bits_per_sample / 8u);

    fs_ret = f_expand(&s_file, target_size, 1);
    if (fs_ret == FR_OK)
    {
        fs_ret = f_lseek(&s_file, 0);
    }

    return fs_ret;
}
static void pcm_ring_reset(void)
{
    s_pcm_ring_read = 0;
    s_pcm_ring_write = 0;
    s_pcm_ring_count = 0;
    g_debug_pcm_ring_max = 0;
    g_debug_pcm_ring_overflow_count = 0;
}

static uint32_t pcm_ring_push(const int16_t *p_samples, uint32_t sample_count)
{
    uint32_t pushed = 0;

    taskENTER_CRITICAL();
    while (pushed < sample_count && s_pcm_ring_count < PCM_RING_SAMPLES)
    {
        s_pcm_ring[s_pcm_ring_write] = p_samples[pushed];
        s_pcm_ring_write = (s_pcm_ring_write + 1u) % PCM_RING_SAMPLES;
        s_pcm_ring_count++;
        pushed++;
    }

    if (pushed < sample_count)
    {
        g_debug_pcm_ring_overflow_count += sample_count - pushed;
    }
    if (s_pcm_ring_count > g_debug_pcm_ring_max)
    {
        g_debug_pcm_ring_max = s_pcm_ring_count;
    }
    taskEXIT_CRITICAL();

    return pushed;
}

static uint32_t pcm_ring_pop(int16_t *p_samples, uint32_t max_samples)
{
    uint32_t popped = 0;

    taskENTER_CRITICAL();
    while (popped < max_samples && s_pcm_ring_count > 0u)
    {
        p_samples[popped] = s_pcm_ring[s_pcm_ring_read];
        s_pcm_ring_read = (s_pcm_ring_read + 1u) % PCM_RING_SAMPLES;
        s_pcm_ring_count--;
        popped++;
    }
    taskEXIT_CRITICAL();
    return popped;
}
audio_storage_status_t audio_storage_init(void)
{
    FRESULT fs_ret;

    if (!s_driver_linked)
    {
        if (fatfs_init_driver(&s_tf_disk_driver, s_drive_path) != 0u)
        {
            APP_LOG_ERROR("TF FatFs driver link failed.");
            g_debug_tf_last_status = AUDIO_STORAGE_ERR_FS;
            return AUDIO_STORAGE_ERR_FS;
        }
        s_driver_linked = true;
        g_debug_tf_recording = 0;
        g_debug_tf_last_status = AUDIO_STORAGE_OK;
    }

    fs_ret = f_mount(&s_fs, s_drive_path, 1);
    if (fs_ret != FR_OK)
    {
        APP_LOG_ERROR("TF mount failed: %d", fs_ret);
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_FS;
        return AUDIO_STORAGE_ERR_FS;
    }

    s_mounted = true;
    g_debug_tf_last_status = AUDIO_STORAGE_OK;
    APP_LOG_INFO("TF mounted at %s", s_drive_path);
    return AUDIO_STORAGE_OK;
}

audio_storage_status_t audio_storage_start(const char *file_name)
{
    return audio_storage_start_wav(WAV_DEFAULT_SAMPLE_RATE,
                                   WAV_DEFAULT_BITS_PER_SAMPLE,
                                   WAV_DEFAULT_CHANNELS,
                                   file_name);
}

audio_storage_status_t audio_storage_start_wav(uint32_t sample_rate,
                                               uint16_t bits_per_sample,
                                               uint16_t channels,
                                               const char *file_name)
{
    return audio_storage_start_wav_for_samples(sample_rate,
                                               bits_per_sample,
                                               channels,
                                               0,
                                               file_name);
}

audio_storage_status_t audio_storage_start_wav_for_samples(uint32_t sample_rate,
                                                           uint16_t bits_per_sample,
                                                           uint16_t channels,
                                                           uint32_t max_samples,
                                                           const char *file_name)
{
    FRESULT fs_ret;
    char name[13];
    uint16_t index;

    if (!s_mounted)
    {
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_NOT_READY;
        return AUDIO_STORAGE_ERR_NOT_READY;
    }

    if (s_recording)
    {
        audio_storage_stop();
    }

    if (file_name != NULL && file_name[0] != '\0')
    {
        build_file_path(s_file_path, file_name);
        fs_ret = f_open(&s_file, s_file_path, FA_WRITE | FA_CREATE_NEW);
    }
    else
    {
        fs_ret = FR_DENIED;
        for (index = 1u; index <= 999u; index++)
        {
            sprintf(name, "AUD%03u.WAV", index);
            build_file_path(s_file_path, name);
            fs_ret = f_open(&s_file, s_file_path, FA_WRITE | FA_CREATE_NEW);
            if (fs_ret == FR_OK)
            {
                break;
            }
            if (fs_ret != FR_EXIST)
            {
                break;
            }
        }
    }

    if (fs_ret != FR_OK)
    {
        if (fs_ret == FR_EXIST)
        {
            g_debug_tf_last_status = AUDIO_STORAGE_ERR_EXISTS;
            return AUDIO_STORAGE_ERR_EXISTS;
        }
        APP_LOG_ERROR("TF file open failed: %d", fs_ret);
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_FILE;
        return AUDIO_STORAGE_ERR_FILE;
    }

    s_wav_sample_rate = sample_rate;
    s_wav_bits_per_sample = bits_per_sample;
    s_wav_channels = channels;
    s_wav_data_bytes = 0;
    s_target_samples = max_samples;
    s_written_samples = 0;
    s_captured_samples = 0;
    s_capture_done = false;
    pcm_ring_reset();
    fs_ret = wav_file_preallocate(max_samples);
    g_debug_tf_expand_fresult = fs_ret;
    fs_ret = wav_header_update();
    if (fs_ret != FR_OK)
    {
        (void)f_close(&s_file);
        APP_LOG_ERROR("WAV header write failed: %d", fs_ret);
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_FILE;
        return AUDIO_STORAGE_ERR_FILE;
    }
    fs_ret = f_sync(&s_file);
    if (fs_ret != FR_OK)
    {
        (void)f_close(&s_file);
        APP_LOG_ERROR("WAV header sync failed: %d", fs_ret);
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_FILE;
        return AUDIO_STORAGE_ERR_FILE;
    }
    fs_ret = f_lseek(&s_file, WAV_HEADER_SIZE);
    if (fs_ret != FR_OK)
    {
        (void)f_close(&s_file);
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_FILE;
        return AUDIO_STORAGE_ERR_FILE;
    }

    s_recording = true;
    s_ble_send_done = false;  // 允许新录音触发 BLE 传输
    s_bytes_since_sync = 0;
    g_debug_tf_recording = 1;
    g_debug_tf_written_bytes = 0;
    g_debug_tf_sync_fresult = FR_OK;
    g_debug_tf_close_fresult = FR_OK;
    g_debug_tf_file_size = f_size(&s_file);
    g_debug_tf_last_status = AUDIO_STORAGE_OK;
    APP_LOG_INFO("TF recording file: %s", s_file_path);
    return AUDIO_STORAGE_OK;
}

audio_storage_status_t audio_storage_write(const void *p_data, uint32_t length)
{
    UINT written;
    FRESULT fs_ret;

    if (!s_recording)
    {
        return AUDIO_STORAGE_ERR_NOT_READY;
    }
    if (p_data == NULL || length == 0u)
    {
        return AUDIO_STORAGE_OK;
    }

    fs_ret = f_write(&s_file, p_data, length, &written);
    if (fs_ret != FR_OK || written != length)
    {
        APP_LOG_ERROR("TF write failed: %d, %u/%u", fs_ret, written, length);
        s_recording = false;
        g_debug_tf_recording = 0;
        g_debug_tf_last_status = AUDIO_STORAGE_ERR_FILE;
        return AUDIO_STORAGE_ERR_FILE;
    }

    s_bytes_since_sync += written;
    s_wav_data_bytes += written;
    g_debug_tf_written_bytes = s_wav_data_bytes;
    return AUDIO_STORAGE_OK;
}

audio_storage_status_t audio_storage_write_pcm16(const int16_t *p_samples, uint32_t sample_count)
{
    uint32_t samples_to_queue = sample_count;
    uint32_t samples_left;
    uint32_t queued;

    if (!s_recording || s_capture_done)
    {
        return AUDIO_STORAGE_ERR_NOT_READY;
    }
    if (p_samples == NULL || sample_count == 0u)
    {
        return AUDIO_STORAGE_OK;
    }

    if (s_target_samples != 0u)
    {
        if (s_captured_samples >= s_target_samples)
        {
            taskENTER_CRITICAL();
            s_capture_done = true;
            taskEXIT_CRITICAL();
            return AUDIO_STORAGE_OK;
        }

        samples_left = s_target_samples - s_captured_samples;
        if (samples_to_queue > samples_left)
        {
            samples_to_queue = samples_left;
        }
    }

    queued = pcm_ring_push(p_samples, samples_to_queue);
    s_captured_samples += queued;

    if (queued < samples_to_queue)
    {
        taskENTER_CRITICAL();
        s_capture_done = true;
        taskEXIT_CRITICAL();
        return AUDIO_STORAGE_ERR_FILE;
    }

    if (s_target_samples != 0u && s_captured_samples >= s_target_samples)
    {
        taskENTER_CRITICAL();
        s_capture_done = true;
        taskEXIT_CRITICAL();
    }

    return AUDIO_STORAGE_OK;
}

void audio_storage_capture_done(void)
{
    taskENTER_CRITICAL();
    s_capture_done = true;
    taskEXIT_CRITICAL();
}

bool audio_storage_is_capture_target_reached(void)
{
    bool done;

    taskENTER_CRITICAL();
    done = s_capture_done;
    taskEXIT_CRITICAL();

    return done;
}

bool audio_storage_is_drain_done(void)
{
    bool done;

    taskENTER_CRITICAL();
    done = s_capture_done && s_pcm_ring_count == 0u;
    taskEXIT_CRITICAL();

    return done;
}

uint32_t audio_storage_captured_samples_get(void)
{
    return s_captured_samples;
}

void audio_storage_process(void)
{
    uint32_t samples;
    bool drain_done;

    if (!s_recording)
    {
        return;
    }

    samples = pcm_ring_pop(s_pcm_write_chunk, PCM_WRITE_CHUNK_SAMPLES);
    if (samples > 0u)
    {
        if (audio_storage_write(s_pcm_write_chunk, samples * sizeof(int16_t)) == AUDIO_STORAGE_OK)
        {
            s_written_samples += samples;
        }
    }

    taskENTER_CRITICAL();
    drain_done = s_capture_done && s_pcm_ring_count == 0u;
    taskEXIT_CRITICAL();

    if (drain_done)
    {
        audio_storage_stop();
    }
}
void audio_storage_stop(void)
{
    if (s_recording)
    {
        FRESULT fs_ret;

        fs_ret = f_lseek(&s_file, (FSIZE_t)WAV_HEADER_SIZE + (FSIZE_t)s_wav_data_bytes);
        if (fs_ret == FR_OK)
        {
            fs_ret = f_truncate(&s_file);
        }
        if (fs_ret == FR_OK)
        {
            fs_ret = wav_header_update();
        }
        g_debug_tf_sync_fresult = fs_ret;
        if (fs_ret == FR_OK)
        {
            fs_ret = f_sync(&s_file);
            g_debug_tf_sync_fresult = fs_ret;
        }
        g_debug_tf_file_size = f_size(&s_file);
        fs_ret = f_close(&s_file);
        g_debug_tf_close_fresult = fs_ret;
        s_recording = false;
        s_bytes_since_sync = 0;
        g_debug_tf_recording = 0;
        g_debug_tf_written_bytes = s_wav_data_bytes;
        APP_LOG_INFO("TF recording stopped.");
    }
}

bool audio_storage_is_recording(void)
{
    return s_recording;
}

const char *audio_storage_file_path_get(void)
{
    return s_file_path;
}

/* ================================================================
 * BLE 文件传输：录制完成后将WAV文件通过BLE发送到手机
 * ================================================================ */
#define BLE_SEND_CHUNK  244u           // MTU-3, 一包 BLE 量

static FIL     s_ble_file;
static bool    s_ble_send_active;
static FSIZE_t s_ble_file_size;
static FSIZE_t s_ble_sent_bytes;
static uint8_t s_ble_read_buf[BLE_SEND_CHUNK];

bool audio_file_transfer_start(const char *path)
{
    FRESULT fr;
    FILINFO info;

    if (s_ble_send_active || s_ble_send_done) return false;
    if (path == NULL) return false;

    fr = f_stat(path, &info);
    if (fr != FR_OK) return false;

    fr = f_open(&s_ble_file, path, FA_READ);
    if (fr != FR_OK) return false;

    s_ble_file_size = info.fsize;
    s_ble_sent_bytes = 0;
    s_ble_send_active = true;
    s_ble_send_done = true;
    transport_set_file_send_active(true);
    transport_ring_flush(); // 清空 ring buffer 中的残留数据

    APP_LOG_INFO("BLE send start: %s (%lu bytes)", path, s_ble_file_size);
    return true;
}

uint32_t audio_file_transfer_process(void)
{
    UINT br;

    if (!s_ble_send_active) return 0;
    if (!transport_flag_cfm(GUS_TX_NTF_ENABLE)) return s_ble_file_size - s_ble_sent_bytes;

    // 多 chunk 推入 ring buffer
    while (ble_ring_has_space(BLE_SEND_CHUNK) && s_ble_sent_bytes < s_ble_file_size)
    {
        if (f_read(&s_ble_file, s_ble_read_buf, BLE_SEND_CHUNK, &br) != FR_OK || br == 0)
        {
            audio_file_transfer_stop();
            return 0;
        }
        uart_to_ble_push(s_ble_read_buf, br);
        s_ble_sent_bytes += br;
    }

    if (s_ble_sent_bytes >= s_ble_file_size)
    {
        audio_file_transfer_stop();
        return 0;
    }
    return s_ble_file_size - s_ble_sent_bytes;
}

void audio_file_transfer_stop(void)
{
    if (s_ble_send_active)
    {
        f_close(&s_ble_file);
        s_ble_send_active = false;
        transport_set_file_send_active(false);
        uart_rx_restart(); // 恢复 UART RX
        APP_LOG_INFO("BLE send done: %lu/%lu bytes", s_ble_sent_bytes, s_ble_file_size);
    }
}
