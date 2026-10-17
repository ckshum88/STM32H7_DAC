#ifndef __BSP_AUDIO_H
#define __BSP_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SAI1 Block A master TX -> external I2S DAC.
   The SAI peripheral and DMA1_Stream0 are configured by CubeMX
   (MX_SAI1_Init / HAL_SAI_MspInit); these helpers manage the playback
   lifecycle and expose the DMA remaining counter used by the USB
   feedback loop in usbd_audio.c. */

/* Per-sample-rate configuration. The SAI1 kernel clock is PLL2-P, chosen
   so that SAI_CK = Fs * 256 (MCKDIV stays 1). fb_nom is the async feedback
   nominal value (10.14 format shifted << 8) for that rate. */
typedef struct
{
  uint32_t freq;       /* sample rate in Hz (44100 / 48000 / 96000) */
  uint32_t pll2n;      /* PLL2 N */
  uint32_t pll2fracn;  /* PLL2 FRACN */
  uint32_t pll2p;      /* PLL2 P */
  uint32_t fb_nom;     /* async feedback nominal (10.14 << 8) */
} AUDIO_RateConfig_t;

extern const AUDIO_RateConfig_t AudioRateConfig[];

const AUDIO_RateConfig_t *BSP_AUDIO_OUT_GetRateConfig(uint32_t AudioFreq);

uint8_t BSP_AUDIO_OUT_Init(uint32_t AudioFreq);
uint8_t BSP_AUDIO_OUT_Play(uint32_t *pbuf, uint16_t size_words);
uint8_t BSP_AUDIO_OUT_Stop(void);
uint8_t BSP_AUDIO_OUT_DeInit(void);

/* Remaining 32-bit words before the circular DMA wraps (NDTR). */
uint32_t BSP_AUDIO_OUT_GetRemainingDataSize(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_AUDIO_H */
