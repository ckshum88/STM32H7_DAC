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

/**
  * @brief  Poll for a fresh FFT spectrum block (display tap).
  *         When a full analysis window has been captured, window -> FFT ->
  *         magnitude -> log-frequency binning is computed and the per-band
  *         magnitudes (0.0 .. 1.0) are written to @p bars.
  *         Must be called from the main loop, never from the USB ISR.
  * @param  bars: output array of per-band magnitudes (0.0 .. 1.0)
  * @param  nbars: number of log-spaced bands (typically 8)
  * @retval 1 if @p bars was refreshed with new data, 0 otherwise
  */
uint8_t AudioDSP_Spectrum(float *bars, uint32_t nbars);

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_DSP_H */
