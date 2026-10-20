#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Quadrature edges per mechanical detent (EC11-style encoders: 4). */
#ifndef BSP_ENC_COUNTS_PER_DETENT
#define BSP_ENC_COUNTS_PER_DETENT 4
#endif

/* Rotation direction: +1 keeps the raw sign, -1 inverts it. */
#ifndef BSP_ENC_DIR_SIGN
#define BSP_ENC_DIR_SIGN (-1)
#endif

/* Button wiring: 1 = pressed pulls the pin low (external pull-up), 0 = active high. */
#ifndef BSP_ENC_BUTTON_ACTIVE_LOW
#define BSP_ENC_BUTTON_ACTIVE_LOW 1
#endif

/**
  * @brief  Bring up the rotary encoder (TIM2 in quadrature encoder mode on
  *         PA0/PA1) and the encoder push button (PA2). Clocks and GPIO are
  *         handled by the local HAL_TIM_Encoder_MspInit() override.
  */
void BSP_ENC_Init(void);

/**
  * @brief  Poll the encoder and return the number of detents turned since the
  *         previous call. Positive = one direction, negative = the other.
  *         Must be called periodically from the main loop.
  */
int32_t BSP_ENC_GetSteps(void);

/**
  * @brief  Poll the push button with debounce.
  * @retval 1 on a single debounced press event (release->press), else 0.
  */
uint8_t BSP_ENC_ButtonEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H */
