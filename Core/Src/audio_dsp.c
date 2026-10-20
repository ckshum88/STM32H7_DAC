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

/* ------------------------------------------------------------------------- */
/* 3-band EQ: RBJ cookbook biquads (low-shelf, peaking, high-shelf).         */
/* ------------------------------------------------------------------------- */

#define DSP_EQ_BANDS         3U
#define DSP_EQ_LOW_SHELF_HZ  200.0f
#define DSP_EQ_MID_HZ        1000.0f
#define DSP_EQ_MID_Q         0.707f
#define DSP_EQ_HIGH_SHELF_HZ 4000.0f

typedef struct
{
  float b0, b1, b2, a1, a2;
} dsp_biquad_t;

typedef struct
{
  float z1, z2;
} dsp_biquad_state_t;

static dsp_biquad_t       eq_coef[DSP_EQ_BANDS];
static dsp_biquad_state_t eq_state[2][DSP_EQ_BANDS];
static int8_t             eq_gain_db[DSP_EQ_BANDS] = {0, 0, 0};
static float              eq_fs = 48000.0f;

/**
  * @brief  Compute one RBJ biquad's normalised coefficients for a given rate
  *         and band gain (dB).
  */
static void eq_compute_band(uint8_t band, float fs, int8_t db, dsp_biquad_t *out)
{
  const float sqrt2 = 1.4142135623730951f;
  float w0, cw, sw, A, sqrtA, alpha;
  float b0, b1, b2, a0, a1, a2;
  float f0;

  A     = powf(10.0f, (float)db / 40.0f);
  sqrtA = sqrtf(A);

  f0 = (band == 0U) ? DSP_EQ_LOW_SHELF_HZ :
       (band == 1U) ? DSP_EQ_MID_HZ : DSP_EQ_HIGH_SHELF_HZ;
  w0 = 2.0f * (float)M_PI * f0 / fs;
  cw = cosf(w0);
  sw = sinf(w0);

  if (band == 1U)
  {
    /* Peaking EQ. */
    alpha = sw / (2.0f * DSP_EQ_MID_Q);
    b0 =  1.0f + (alpha * A);
    b1 = -2.0f * cw;
    b2 =  1.0f - (alpha * A);
    a0 =  1.0f + (alpha / A);
    a1 = -2.0f * cw;
    a2 =  1.0f - (alpha / A);
  }
  else
  {
    /* Shelf (slope S = 1). */
    float twoSqrtAalpha;

    alpha = (sw * 0.5f) * sqrt2;
    twoSqrtAalpha = 2.0f * sqrtA * alpha;

    if (band == 0U)
    {
      /* Low shelf. */
      b0 =      A * ((A + 1.0f) - (A - 1.0f) * cw + twoSqrtAalpha);
      b1 =  2.0f * A * ((A - 1.0f) - (A + 1.0f) * cw);
      b2 =      A * ((A + 1.0f) - (A - 1.0f) * cw - twoSqrtAalpha);
      a0 =           (A + 1.0f) + (A - 1.0f) * cw + twoSqrtAalpha;
      a1 = -2.0f *  ((A - 1.0f) + (A + 1.0f) * cw);
      a2 =           (A + 1.0f) + (A - 1.0f) * cw - twoSqrtAalpha;
    }
    else
    {
      /* High shelf. */
      b0 =      A * ((A + 1.0f) + (A - 1.0f) * cw + twoSqrtAalpha);
      b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cw);
      b2 =      A * ((A + 1.0f) + (A - 1.0f) * cw - twoSqrtAalpha);
      a0 =           (A + 1.0f) - (A - 1.0f) * cw + twoSqrtAalpha;
      a1 =  2.0f *  ((A - 1.0f) - (A + 1.0f) * cw);
      a2 =           (A + 1.0f) - (A - 1.0f) * cw - twoSqrtAalpha;
    }
  }

  out->b0 = b0 / a0;
  out->b1 = b1 / a0;
  out->b2 = b2 / a0;
  out->a1 = a1 / a0;
  out->a2 = a2 / a0;
}

/**
  * @brief  Set an EQ band gain. The sample-rate read and coefficient commit
  *         happen under one critical section so a concurrent rate change
  *         (AudioDSP_Init, USB control path) cannot interleave.
  */
void AudioDSP_SetEqGainDb(uint8_t band, int8_t db)
{
  dsp_biquad_t c;
  uint32_t primask;

  if (band >= DSP_EQ_BANDS)
  {
    return;
  }
  if (db < (int8_t)AUDIO_DSP_EQ_DB_MIN) db = (int8_t)AUDIO_DSP_EQ_DB_MIN;
  if (db > (int8_t)AUDIO_DSP_EQ_DB_MAX) db = (int8_t)AUDIO_DSP_EQ_DB_MAX;

  primask = __get_PRIMASK();
  __disable_irq();

  eq_gain_db[band] = db;
  eq_compute_band(band, eq_fs, db, &c);
  eq_coef[band] = c;

  __set_PRIMASK(primask);
}

/**
  * @brief  Read back an EQ band gain (dB).
  */
int8_t AudioDSP_GetEqGainDb(uint8_t band)
{
  if (band >= DSP_EQ_BANDS)
  {
    return 0;
  }
  return eq_gain_db[band];
}

/**
  * @brief  Initialize the DSP stage: rebuild the analysis window and reset
  *         the display tap. Called on startup and on every sample-rate change.
  */
void AudioDSP_Init(uint32_t sample_rate)
{
  uint32_t i;
  uint32_t ch;
  uint32_t b;
  uint32_t n = DSP_FFT_N;

  tap_active = 0U;
  tap_fill   = 0U;
  tap_ready  = 0U;

  eq_fs = (float)sample_rate;

  /* Hann window. */
  for (i = 0; i < n; i++)
  {
    win[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1U));
  }

  /* Bit-reversal table for the radix-2 FFT (log2(n) = 9 for n = 512). */
  for (i = 0; i < n; i++)
  {
    uint32_t r = 0U;
    uint32_t b2 = i;
    uint32_t j;
    for (j = 0; j < 9U; j++)
    {
      r = (r << 1) | (b2 & 1U);
      b2 >>= 1;
    }
    rev[i] = (uint16_t)r;
  }

  /* EQ: rebuild coefficients for the new rate from the persisted gains and
     clear the per-channel filter state. Runs in the USB control path, which is
     already interrupt context, so a plain store is safe. */
  for (b = 0U; b < DSP_EQ_BANDS; b++)
  {
    eq_compute_band((uint8_t)b, eq_fs, eq_gain_db[b], &eq_coef[b]);
  }
  for (ch = 0U; ch < 2U; ch++)
  {
    for (b = 0U; b < DSP_EQ_BANDS; b++)
    {
      eq_state[ch][b].z1 = 0.0f;
      eq_state[ch][b].z2 = 0.0f;
    }
  }
}

/**
  * @brief  Pass-through DSP stage. Besides the (no-op) audio processing it
  *         taps a mono downmix into the spectrum analysis buffer.
  */
void AudioDSP_ProcessBlock(float *pcm, uint32_t nframes)
{
  uint32_t i;
  uint32_t ch;
  uint32_t b;

  for (i = 0; i < nframes; i++)
  {
    /* 3-band EQ per channel (transposed direct-form II biquads). */
    for (ch = 0U; ch < 2U; ch++)
    {
      float x = pcm[2U * i + ch];

      for (b = 0U; b < DSP_EQ_BANDS; b++)
      {
        dsp_biquad_state_t *s = &eq_state[ch][b];
        const dsp_biquad_t *c = &eq_coef[b];
        float y = (c->b0 * x) + s->z1;
        s->z1 = (c->b1 * x) - (c->a1 * y) + s->z2;
        s->z2 = (c->b2 * x) - (c->a2 * y);
        x = y;
      }

      pcm[2U * i + ch] = x;
    }

    /* Spectrum tap: mono downmix of the EQ'd samples (post-EQ). */
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
