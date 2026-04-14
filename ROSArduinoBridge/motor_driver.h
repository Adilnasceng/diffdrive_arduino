/***************************************************************
   Motor driver function definitions — STM32F407G-DISC1 versiyonu
   BTS7960 motor sürücüsü · TIM1 PWM çıkışları (PE9/PE11/PE13/PE14)
   *************************************************************/

#ifdef BTS7960_MOTOR_DRIVER
  /* Sol Motor — TIM1_CH1 / TIM1_CH2
   *   PE9  → TIM1_CH1_N  (AF1) → L_PWM (ileri)
   *   PE11 → TIM1_CH2_N  (AF1) → R_PWM (geri)
   */
  #define LEFT_MOTOR_FORWARD    PE9
  #define LEFT_MOTOR_BACKWARD   PE11

  /* Sağ Motor — TIM1_CH3 / TIM1_CH4
   *   PE13 → TIM1_CH3    (AF1) → L_PWM (ileri)
   *   PE14 → TIM1_CH4    (AF1) → R_PWM (geri)
   */
  #define RIGHT_MOTOR_FORWARD   PE13
  #define RIGHT_MOTOR_BACKWARD  PE14

  /* BTS7960 enable pinleri donanımsal olarak VCC'ye bağlı,
     yazılımdan kontrol gerekmez */
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
