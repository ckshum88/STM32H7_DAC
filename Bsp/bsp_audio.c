#include "bsp_audio.h"

extern SAI_HandleTypeDef hsai_BlockA1;

/* PLL2 input is HSE (25 MHz) / M = 1.190476 MHz. SAI1_CK = PLL2-P is chosen
   per rate so that Fs = SAI1_CK / 256 with the SAI MCKDIV = 1
   (I2S standard, 32-bit, 2 slots, NODIV = 0, no MCK oversampling).

   fb_nom = Fs/1000 * 2^22 (samples-per-frame in 10.14, shifted << 8). The
   PLL2 FRACN values below give near-exact rates, so the nominal value is used
   directly; the residual error is well below the 10.14 feedback resolution.

     44.1k: VCO = 25/21*(265 + 4353/8192) = 316.109 MHz, /28 = 11.2896 MHz,
            Fs = 11.2896 MHz / 256 = 44.09999 kHz
     48k:   VCO = 25/21*(289 + 113/8192)  = 344.064 MHz, /28 = 12.2880 MHz,
            Fs = 12.2880 MHz / 256 = 48.00001 kHz
     96k:   VCO = 344.064 MHz (unchanged), /14 = 24.5760 MHz,
            Fs = 24.5760 MHz / 256 = 96.00001 kHz */
const AUDIO_RateConfig_t AudioRateConfig[] =
{
  { 44100U, 265U, 4353U, 28U, 0x0B066666U },
  { 48000U, 289U,  113U, 28U, 0x0C000000U },
  { 96000U, 289U,  113U, 14U, 0x18000000U },
};

const AUDIO_RateConfig_t *BSP_AUDIO_OUT_GetRateConfig(uint32_t AudioFreq)
{
  uint32_t i;
  for (i = 0; i < (uint32_t)(sizeof(AudioRateConfig) / sizeof(AudioRateConfig[0])); i++)
  {
    if (AudioRateConfig[i].freq == AudioFreq)
    {
      return &AudioRateConfig[i];
    }
  }
  return NULL;
}

/**
  * @brief  Initialize the audio output path for the requested sample rate:
  *         reconfigure PLL2 (SAI1 kernel clock) and re-run the SAI protocol
  *         init so the MCKDIV is recomputed for the new frequency.
  * @param  AudioFreq: 44100, 48000 or 96000
  * @retval 0 = success, 1 = error
  */
uint8_t BSP_AUDIO_OUT_Init(uint32_t AudioFreq)
{
  const AUDIO_RateConfig_t *cfg = BSP_AUDIO_OUT_GetRateConfig(AudioFreq);
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (cfg == NULL)
  {
    return 1U;
  }

  /* Reconfigure PLL2 P output (SAI1 kernel clock). The SAI must be stopped
     (caller already did HAL_SAI_DMAStop) while the PLL re-locks. */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL2.PLL2M = 21U;
  PeriphClkInitStruct.PLL2.PLL2N = cfg->pll2n;
  PeriphClkInitStruct.PLL2.PLL2P = cfg->pll2p;
  PeriphClkInitStruct.PLL2.PLL2Q = 2U;
  PeriphClkInitStruct.PLL2.PLL2R = 2U;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = cfg->pll2fracn;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    return 1U;
  }

  /* Recompute the SAI MCKDIV for the new kernel clock and audio frequency. */
  hsai_BlockA1.Init.AudioFrequency = AudioFreq;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2U) != HAL_OK)
  {
    return 1U;
  }

  return 0U;
}

/**
  * @brief  Start/restart circular DMA playback of 32-bit I2S words.
  * @param  pbuf: pointer to the ring buffer
  * @param  size_words: number of 32-bit words in the circular transfer
  */
uint8_t BSP_AUDIO_OUT_Play(uint32_t *pbuf, uint16_t size_words)
{
  if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)pbuf, size_words) != HAL_OK)
  {
    return 1U;
  }
  return 0U;
}

/**
  * @brief  Stop SAI DMA playback (disables SAI + aborts the DMA stream).
  */
uint8_t BSP_AUDIO_OUT_Stop(void)
{
  (void)HAL_SAI_DMAStop(&hsai_BlockA1);
  return 0U;
}

/**
  * @brief  De-initialize the audio output path.
  */
uint8_t BSP_AUDIO_OUT_DeInit(void)
{
  return BSP_AUDIO_OUT_Stop();
}

/**
  * @brief  Remaining 32-bit words before the circular DMA wraps (NDTR).
  *         Used by the USB feedback loop to derive the read pointer.
  */
uint32_t BSP_AUDIO_OUT_GetRemainingDataSize(void)
{
  return (uint32_t)__HAL_DMA_GET_COUNTER(hsai_BlockA1.hdmatx);
}
