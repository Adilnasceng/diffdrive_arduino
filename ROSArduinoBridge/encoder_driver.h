/* *************************************************************
   Encoder driver function definitions — STM32F407G-DISC1 versiyonu
   Donanım encoder modu: TIM2 (sol, 32-bit) / TIM3 (sağ, 16-bit)
   ************************************************************ */

#ifdef STM32_HW_ENCODER
  /* Sol encoder — TIM2 (32-bit timer)
   *   CH1 / A faz : PA15  (TIM2_CH1, AF1)
   *   CH2 / B faz : PB3   (TIM2_CH2, AF1)
   */
  #define LEFT_ENC_CH1   PA15
  #define LEFT_ENC_CH2   PB3

  /* Sağ encoder — TIM3 (16-bit timer + yazılım akümülasyonu)
   *   CH1 / A faz : PB4   (TIM3_CH1, AF2)
   *   CH2 / B faz : PB5   (TIM3_CH2, AF2)
   */
  #define RIGHT_ENC_CH1  PB4
  #define RIGHT_ENC_CH2  PB5
#endif

void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
