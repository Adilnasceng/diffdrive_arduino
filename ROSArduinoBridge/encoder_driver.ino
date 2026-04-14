/* *************************************************************
   STM32F407G-DISC1 Donanım Encoder Sürücüsü

   TIM2 — Sol encoder  (32-bit): PA15 (CH1/A) · PB3  (CH2/B)
   TIM3 — Sağ encoder  (16-bit): PB4  (CH1/A) · PB5  (CH2/B)

   Encoder modu : TIM_ENCODERMODE_TI12 (her iki fazda sayım → 4x çözünürlük)
   Gürültü filtresi: IC1/IC2Filter = 0x0A (8 örnek, APB1/16)

   TIM2 32-bit olduğu için taşma pratikte yaşanmaz; doğrudan okunur.
   TIM3 16-bit olduğu için int16_t fark hilesiyle 32-bit akümüle edilir;
   noInterrupts()/interrupts() ile yarış koşuluna karşı korunur.
   ************************************************************ */

#ifdef USE_BASE
#ifdef STM32_HW_ENCODER

#include "stm32f4xx_hal.h"

/* ---------- Timer handle'ları ---------- */
static TIM_HandleTypeDef htim2;   /* Sol  — 32-bit */
static TIM_HandleTypeDef htim3;   /* Sağ  — 16-bit */

/* ---------- Sol encoder değişkenleri ---------- */
/* TIM2 32-bit sayar; sıfırlama için sayaç yerine ofset saklanır */
static volatile int32_t left_enc_offset = 0;

/* ---------- Sağ encoder değişkenleri ---------- */
/* TIM3 16-bit → int16_t fark hilesiyle 32-bit'e genişletilir */
static volatile int32_t  right_enc_pos  = 0;
static volatile uint16_t right_enc_last = 0;

/* TIM3 akümülatörünü güncelle (yarış koşulsuz bağlamda çağrılmalı) */
static void updateRightEncoder() {
  uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  /* int16_t cast ile taşma/altüsya otomatik yakalanır (±32767 tick/çağrı sınırı) */
  right_enc_pos += (int32_t)(int16_t)(current - right_enc_last);
  right_enc_last = current;
}

/* ---- initEncoders ---- */
void initEncoders() {
  GPIO_InitTypeDef     GPIO_InitStruct = {0};
  TIM_Encoder_InitTypeDef sConfig      = {0};

  /* Ortak encoder yapılandırması */
  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 0x0A;   /* gürültü filtresi */
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 0x0A;

  /* ================================================================
     TIM2 — Sol encoder  (PA15 / PB3)
     ================================================================ */
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA15 → TIM2_CH1 (AF1) */
  GPIO_InitStruct.Pin       = GPIO_PIN_15;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB3 → TIM2_CH2 (AF1) */
  GPIO_InitStruct.Pin       = GPIO_PIN_3;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 0xFFFFFFFF;   /* 32-bit tam aralık */
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim2, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  left_enc_offset = 0;

  /* ================================================================
     TIM3 — Sağ encoder  (PB4 / PB5)
     ================================================================ */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* PB4 → TIM3_CH1 (AF2) */
  GPIO_InitStruct.Pin       = GPIO_PIN_4;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB5 → TIM3_CH2 (AF2) */
  GPIO_InitStruct.Pin       = GPIO_PIN_5;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 0xFFFF;   /* 16-bit tam aralık */
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  right_enc_pos  = 0;
  right_enc_last = 0;
}

/* ---- readEncoder ---- */
long readEncoder(int i) {
  if (i == LEFT) {
    /* TIM2 32-bit: doğrudan oku, sıfırlama ofseti çıkar */
    int32_t raw = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return (long)(raw - left_enc_offset);
  } else {
    /* TIM3 16-bit: önce akümüle et, sonra oku */
    noInterrupts();
    updateRightEncoder();
    long val = (long)right_enc_pos;
    interrupts();
    return val;
  }
}

/* ---- resetEncoder ---- */
void resetEncoder(int i) {
  if (i == LEFT) {
    /* Sayacı durdurmak yerine ofseti güncelle — TIM2 kesintisiz çalışır */
    left_enc_offset = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  } else {
    noInterrupts();
    updateRightEncoder();           /* mevcut durumu önce akümüle et */
    right_enc_pos  = 0;
    right_enc_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    interrupts();
  }
}

/* ---- resetEncoders ---- */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#else
  #error An encoder driver must be selected! (Define STM32_HW_ENCODER)
#endif  /* STM32_HW_ENCODER */
#endif  /* USE_BASE */
