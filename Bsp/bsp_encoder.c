#include "bsp_encoder.h"
#include "main.h"

#define ENC_BTN_DEBOUNCE_SAMPLES 2U

extern TIM_HandleTypeDef htim2;

static uint32_t enc_last     = 0U;
static int32_t  enc_residual = 0;

static uint8_t  btn_stable      = 0U;   /* debounced state, 1 = pressed */
static uint8_t  btn_prev_stable = 0U;
static uint8_t  btn_count       = 0U;

static uint8_t enc_button_raw(void)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin);

#if (BSP_ENC_BUTTON_ACTIVE_LOW != 0)
  return (state == GPIO_PIN_RESET) ? 1U : 0U;
#else
  return (state == GPIO_PIN_SET) ? 1U : 0U;
#endif
}

void BSP_ENC_Init(void)
{
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  enc_last        = __HAL_TIM_GET_COUNTER(&htim2);
  enc_residual    = 0;
  btn_stable      = enc_button_raw();
  btn_prev_stable = btn_stable;
  btn_count       = 0U;
}

int32_t BSP_ENC_GetSteps(void)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
  int32_t  delta = (int32_t)(now - enc_last);
  int32_t  steps;

  enc_last = now;

  delta *= (int32_t)BSP_ENC_DIR_SIGN;

  enc_residual += delta;
  steps = enc_residual / BSP_ENC_COUNTS_PER_DETENT;
  enc_residual -= steps * BSP_ENC_COUNTS_PER_DETENT;

  return steps;
}

uint8_t BSP_ENC_ButtonEvent(void)
{
  uint8_t raw = enc_button_raw();
  uint8_t event = 0U;

  if (raw != btn_stable)
  {
    if (btn_count < 0xFFU)
    {
      btn_count++;
    }
    if (btn_count >= ENC_BTN_DEBOUNCE_SAMPLES)
    {
      btn_stable = raw;
      btn_count  = 0U;
    }
  }
  else
  {
    btn_count = 0U;
  }

  if ((btn_stable != 0U) && (btn_prev_stable == 0U))
  {
    event = 1U;
  }
  btn_prev_stable = btn_stable;

  return event;
}
