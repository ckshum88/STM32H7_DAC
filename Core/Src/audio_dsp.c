#include "audio_dsp.h"

/**
  * @brief  Initialize the DSP stage (no state needed for pass-through).
  */
void AudioDSP_Init(uint32_t sample_rate)
{
  (void)sample_rate;
}

/**
  * @brief  Pass-through DSP stage. Replace the body with the desired
  *         processing (e.g. biquad filters, EQ) operating on interleaved
  *         L/R float32 samples in [-1.0, 1.0).
  */
void AudioDSP_ProcessBlock(float *pcm, uint32_t nframes)
{
  (void)pcm;
  (void)nframes;
}
