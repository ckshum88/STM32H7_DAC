#include "bsp_encoder.h"
#include "main.h"

#define ENC_SW_Pin        GPIO_PIN_2
#define ENC_SW_GPIO_Port  GPIOA

#define ENC_BTN_DEBOUNCE_SAMPLES 2U

static TIM_HandleTypeDef htim2;

static uint32_t enc_last     = 0U;
static int32_t  enc_residual = 0;

static uint8_t  btn_stable      = 0U;   /* debounced state, 1 = pressed */
static uint8_t  btn_prev_stable = 0U;
static uint8_t  btn_count       = 0U;

/**
  * @brief  Low-level init for TIM2 encoder mode: clocks and PA0/PA1/P2 GPIO.
  *         Overrides the weak HAL callback so no CubemxM MSP entry is required.
  */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};

  if (htim->Instance != TIM2)
  {
    return;
  }

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA0 = TIM2_CH1 (TI1), PA1 = TIM2_CH2 (TI2). */
  gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PA2 = encoder push button (external pull-up). */
  gpio.Pin       = ENC_SW_Pin;
  gpio.Mode      = GPIO_MODE_INPUT;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(ENC_SW_GPIO_Port, &gpio);
}

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
  TIM_Encoder_InitTypeDef sConfig = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0U;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 0xFFFFFFFFU;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim2.Init.RepetitionCounter = 0U;

  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 0x0FU;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 0x0FU;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
