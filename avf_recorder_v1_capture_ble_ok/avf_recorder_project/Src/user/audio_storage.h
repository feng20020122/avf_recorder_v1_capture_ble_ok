#ifndef AUDIO_STORAGE_H
#define AUDIO_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    AUDIO_STORAGE_OK = 0,
    AUDIO_STORAGE_ERR_SPI,
    AUDIO_STORAGE_ERR_CARD,
    AUDIO_STORAGE_ERR_FS,
    AUDIO_STORAGE_ERR_FILE,
    AUDIO_STORAGE_ERR_EXISTS,
    AUDIO_STORAGE_ERR_NOT_READY,
} audio_storage_status_t;

audio_storage_status_t audio_storage_init(void);
audio_storage_status_t audio_storage_start(const char *file_name);
audio_storage_status_t audio_storage_start_wav(uint32_t sample_rate,
                                               uint16_t bits_per_sample,
                                               uint16_t channels,
                                               const char *file_name);
audio_storage_status_t audio_storage_start_wav_for_samples(uint32_t sample_rate,
                                                           uint16_t bits_per_sample,
                                                           uint16_t channels,
                                                           uint32_t max_samples,
                                                           const char *file_name);
audio_storage_status_t audio_storage_write(const void *p_data, uint32_t length);
audio_storage_status_t audio_storage_write_pcm16(const int16_t *p_samples, uint32_t sample_count);
void audio_storage_capture_done(void);
bool audio_storage_is_capture_target_reached(void);
bool audio_storage_is_drain_done(void);
uint32_t audio_storage_captured_samples_get(void);
void audio_storage_process(void);
void audio_storage_stop(void);
bool audio_storage_is_recording(void);
const char *audio_storage_file_path_get(void);

#ifdef __cplusplus
}
#endif

#endif
