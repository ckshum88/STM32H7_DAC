#include "audio_dsp.h"
#include "stm32h7xx.h"
#include <math.h>

/* Analysis window size (mono frames) for the spectrum display tap. */
#define DSP_FFT_N    512U
#define DSP_FFT_BINS (DSP_FFT_N / 2U)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Display tap state: ping-pong buffers filled from the audio ISR, drained
   from the main loop. CPU-only (no DMA), so no cache maintenance needed. */
static float tap[2][DSP_FFT_N];
static uint8_t  tap_active = 0U;
static uint32_t tap_fill   = 0U;
static volatile uint8_t tap_ready = 0U;

/* Hann window and bit-reversal table, built in AudioDSP_Init(). */
static float    win[DSP_FFT_N];
static uint16_t rev[DSP_FFT_N];

/* FFT work buffers. */
static float fft_re[DSP_FFT_N];
static float fft_im[DSP_FFT_N];

/**
  * @brief  Initialize the DSP stage: rebuild the analysis window and reset
  *         the display tap. Called on startup and on every sample-rate change.
  */
void AudioDSP_Init(uint32_t sample_rate)
{
  uint32_t i;
  uint32_t n = DSP_FFT_N;

  (void)sample_rate;

  tap_active = 0U;
  tap_fill   = 0U;
  tap_ready  = 0U;

  /* Hann window. */
  for (i = 0; i < n; i++)
  {
    win[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1U));
  }

  /* Bit-reversal table for the radix-2 FFT (log2(n) = 9 for n = 512). */
  for (i = 0; i < n; i++)
  {
    uint32_t r = 0U;
    uint32_t b = i;
    uint32_t j;
    for (j = 0; j < 9U; j++)
    {
      r = (r << 1) | (b & 1U);
      b >>= 1;
    }
    rev[i] = (uint16_t)r;
  }
}

/**
  * @brief  Pass-through DSP stage. Besides the (no-op) audio processing it
  *         taps a mono downmix into the spectrum analysis buffer.
  */
void AudioDSP_ProcessBlock(float *pcm, uint32_t nframes)
{
  uint32_t i;

  for (i = 0; i < nframes; i++)
  {
    if (tap_ready == 0U)
    {
      tap[tap_active][tap_fill++] = 0.5f * (pcm[2U * i] + pcm[2U * i + 1U]);

      if (tap_fill >= DSP_FFT_N)
      {
        tap_fill  = 0U;
        tap_active ^= 1U;
        __DMB();
        tap_ready = 1U;
      }
    }
  }
}

/**
  * @brief  Iterative radix-2 complex FFT (decimation in time, in place).
  */
static void fft_radix2(float *re, float *im, uint32_t n)
{
  uint32_t i, len, j;

  /* Bit-reversal permutation. */
  for (i = 0; i < n; i++)
  {
    uint16_t r = rev[i];
    if (i < (uint32_t)r)
    {
      float tr = re[i]; re[i] = re[r]; re[r] = tr;
      float ti = im[i]; im[i] = im[r]; im[r] = ti;
    }
  }

  /* Butterfly stages. */
  for (len = 2U; len <= n; len <<= 1)
  {
    float ang = -2.0f * (float)M_PI / (float)len;
    float wlen_re = cosf(ang);
    float wlen_im = sinf(ang);
    uint32_t half = len >> 1;

    for (i = 0; i < n; i += len)
    {
      float w_re = 1.0f;
      float w_im = 0.0f;

      for (j = 0; j < half; j++)
      {
        uint32_t a = i + j;
        uint32_t b = i + j + half;
        float u_re = re[a];
        float u_im = im[a];
        float v_re = re[b] * w_re - im[b] * w_im;
        float v_im = re[b] * w_im + im[b] * w_re;
        float t;

        re[a] = u_re + v_re;
        im[a] = u_im + v_im;
        re[b] = u_re - v_re;
        im[b] = u_im - v_im;

        t = w_re * wlen_re - w_im * wlen_im;
        w_im = w_re * wlen_im + w_im * wlen_re;
        w_re = t;
      }
    }
  }
}

/**
  * @brief  Poll the spectrum tap and, if fresh data is available, compute
  *         windowed FFT magnitudes and bin them into log-spaced bands.
  */
uint8_t AudioDSP_Spectrum(float *bars, uint32_t nbars)
{
  const float *src;
  float peak[32];
  float ref;
  uint32_t i, b;

  if (tap_ready == 0U || nbars == 0U || nbars > 32U)
  {
    return 0U;
  }

  src = tap[tap_active ^ 1U];

  /* Windowed copy into the FFT buffers (real input, zero imaginary). */
  for (i = 0; i < DSP_FFT_N; i++)
  {
    fft_re[i] = src[i] * win[i];
    fft_im[i] = 0.0f;
  }

  fft_radix2(fft_re, fft_im, DSP_FFT_N);

  /* Release the tap buffer before the (potentially slow) binning so the
     audio ISR can resume filling the other buffer. */
  tap_ready = 0U;

  for (b = 0; b < nbars; b++)
  {
    peak[b] = 0.0f;
  }

  /* Magnitude per band (skip DC bin). Log-spaced band index over 1..N/2-1. */
  {
    float logmax = logf((float)DSP_FFT_BINS);
    for (i = 1U; i < DSP_FFT_BINS; i++)
    {
      float mag = sqrtf(fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i]);
      float bf = (float)nbars * logf((float)i) / logmax;
      uint32_t bi = (uint32_t)bf;
      if (bi >= nbars) bi = nbars - 1U;
      if (mag > peak[bi]) peak[bi] = mag;
    }
  }

  /* Reference magnitude for a full-scale bin under the Hann window:
     coherent gain ~0.5 -> N/2 * 0.5 = N/4. */
  ref = (float)(DSP_FFT_N / 4U);

  /* Peak magnitude -> dB -> normalized 0..1 over a -60 dB range. */
  for (b = 0; b < nbars; b++)
  {
    float db;
    if (peak[b] <= 0.0f)
    {
      bars[b] = 0.0f;
      continue;
    }
    db = 20.0f * log10f(peak[b] / ref);
    if (db < -60.0f) db = -60.0f;
    if (db > 0.0f)   db = 0.0f;
    bars[b] = (db + 60.0f) / 60.0f;
  }

  return 1U;
}
