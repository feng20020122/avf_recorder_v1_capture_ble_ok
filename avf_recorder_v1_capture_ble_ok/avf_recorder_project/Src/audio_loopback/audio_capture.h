#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include "app_i2s.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_SAMPLE_RATE    8000u
#define AUDIO_FRAME_SAMPLES  1024u

void audio_hardware_init(void);
uint16_t audio_capture_start(void);
void audio_capture_stop(void);
bool audio_capture_is_active(void);
void audio_capture_process(void);

void app_i2s_dma_rx_cplt_callback(app_i2s_evt_t *p_evt);

#ifdef __cplusplus
}
#endif

#endif
