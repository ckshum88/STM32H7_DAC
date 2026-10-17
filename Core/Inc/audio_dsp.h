#ifndef __AUDIO_DSP_H
#define __AUDIO_DSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  Initialize the DSP stage.
  * @param  sample_rate: audio sample rate in Hz
  */
void AudioDSP_Init(uint32_t sample_rate);

/**
  * @brief  Process a block of interleaved stereo samples in float32.
  *         Default implementation is a pass-through. Override/replace the
  *         implementation to add filtering, EQ, etc. without touching the
  *         USB or SAI code.
  * @param  pcm: interleaved L/R samples in [-1.0, 1.0)
  * @param  nframes: number of stereo frames
  */
void AudioDSP_ProcessBlock(float *pcm, uint32_t nframes);

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_DSP_H */
