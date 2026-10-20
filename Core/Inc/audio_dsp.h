#ifndef __AUDIO_DSP_H
#define __AUDIO_DSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 3-band equalizer: Low (low-shelf), Mid (peaking), High (high-shelf). */
#define AUDIO_DSP_EQ_BANDS  3U
#define AUDIO_DSP_EQ_DB_MIN (-15)
#define AUDIO_DSP_EQ_DB_MAX ( 15)

/**
  * @brief  Initialize the DSP stage.
  * @param  sample_rate: audio sample rate in Hz
  */
void AudioDSP_Init(uint32_t sample_rate);

/**
  * @brief  Set the gain of one EQ band.
  *         Safe to call from the main loop while audio is streaming; the
  *         coefficient update is atomic with respect to the USB audio ISR.
  * @param  band: 0 = Low, 1 = Mid, 2 = High
  * @param  db:   gain in dB, clamped to [-15, +15]
  */
void AudioDSP_SetEqGainDb(uint8_t band, int8_t db);

/**
  * @brief  Read back the current gain of one EQ band.
  * @param  band: 0 = Low, 1 = Mid, 2 = High
  * @retval gain in dB in [-15, +15]
  */
int8_t AudioDSP_GetEqGainDb(uint8_t band);

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
