# Embedded Audio Recorder Structure

This project currently has one verified main path:

1. A key event starts a fixed-length recording.
2. GR5515 I2S captures microphone samples.
3. Captured stereo 32-bit I2S frames are reduced to 8 kHz, 16-bit, mono PCM.
4. PCM is written to a WAV file on the TF card.
5. The same PCM stream is pushed over BLE in real time.
6. The Flutter app receives the raw PCM stream and writes a WAV file.

## Runtime Tasks

`audio_task` runs the time-critical recorder path:

- `record_key_process()` handles short press and long press state.
- `audio_capture_process()` drains completed I2S DMA buffers.
- `audio_storage_process()` drains the PCM ring buffer to the TF card.

`system_task` runs background work only when recording is idle:

- log flush
- power management
- BLE transport scheduler
- optional saved-file transfer process

This separation keeps recording stable by avoiding heavier BLE/file-transfer work while I2S capture is active.

## Module Boundaries

`Src/audio_loopback/audio_capture.h`

- Public capture interface used by application code.
- Application files should include this header instead of a codec-specific header.

`Src/audio_loopback/audio_capture_hw.c`

- Current implementation of the public capture interface.
- Still contains WM8962 init/debug code and GR5515 I2S capture code.
- When ES8388 is added, keep the public `audio_capture_*` API stable and replace only the codec-specific init/output path.

`Src/user/audio_storage.h`

- TF card, FatFs, WAV creation, PCM ring buffer, and recording state.

`Src/user/audio_file_transfer.h`

- Optional path for sending an already-saved WAV file through BLE.
- This is separate from the verified real-time PCM stream used by the app today.

`Src/user/transport_scheduler.h`

- BLE/UART ring buffers and BLE send scheduling.
- The app currently receives real-time raw PCM pushed from `audio_capture_process()`.

`Src/user/record_key.c`

- User workflow state machine.
- Short press changes collection position.
- Long press starts a 10-second recording.

## Current BLE Audio Format

- Payload: raw PCM bytes
- Sample rate: 8000 Hz
- Channels: 1
- Bits per sample: 16
- Byte order: little-endian
- WAV header: generated on the phone app side

## ES8388 Replacement Plan

Keep these stable:

- `record_key.c`
- `audio_storage.c`
- `transport_scheduler.c`
- Flutter BLE receive/WAV creation path
- `audio_capture.h` public API

Expected replacement area:

- codec init and output code currently inside `wm8962_inmp441.c`
- possibly rename or split the implementation after ES8388 output is verified

Recommended ES8388 bring-up order:

1. I2C probe and register read/write.
2. Confirm MCLK/BCLK/LRCLK/DOUT/DIN wiring.
3. Output a simple I2S tone to ES8388 headphones.
4. Reconnect the existing capture/storage/BLE path without changing its format.
