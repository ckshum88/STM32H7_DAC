#include "bsp_audio.h"

extern SAI_HandleTypeDef hsai_BlockA1;

/**
  * @brief  Initialize the audio output path (SAI1 Block A, already configured
  *         by MX_SAI1_Init). Fixed 48 kHz for now, so nothing to reprogram.
  */
uint8_t BSP_AUDIO_OUT_Init(uint32_t AudioFreq)
{
  (void)AudioFreq;
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
