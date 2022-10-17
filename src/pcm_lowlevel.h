#ifndef PCM_LOWLEVEL_H
#define PCM_LOWLEVEL_H

#include <Arduino.h>

#define PCM_OK 0
#define PCM_ERROR -1

/* Audio Frequencies */
#ifndef AUDIO_FREQUENCY_16K
  #define AUDIO_FREQUENCY_16K     16000U
#endif

#ifdef __cplusplus
extern "C" {
#endif

int pcm_lowlevel_init(uint32_t sampleRate);
int pcm_lowlevel_deinit();
int pcm_lowlevel_start();
int pcm_lowlevel_stop();

#ifdef __cplusplus
}
#endif

#endif // PCM_LOWLEVEL_H
