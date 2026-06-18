#ifndef AUDIO_FILE_TRANSFER_H
#define AUDIO_FILE_TRANSFER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool audio_file_transfer_start(const char *path);
uint32_t audio_file_transfer_process(void);
void audio_file_transfer_stop(void);

#ifdef __cplusplus
}
#endif

#endif
