/***************************************************************
   Motor driver definitions — STM32F407G-DISC1 versiyonu

   BTS7960 motor sürücüsü için STM32 TIM1 PWM çıkışları.
   PE9 / PE11 → Sol motor ileri/geri  (TIM1_CH1 / TIM1_CH2, AF1)
   PE13 / PE14 → Sağ motor ileri/geri (TIM1_CH3 / TIM1_CH4, AF1)

   PWM frekansı: 20 kHz — işitilebilir titreşimi önler,
   BTS7960 maksimum PWM frekansı (25 kHz) altında güvenli çalışır.
   *************************************************************/

#ifdef USE_BASE

#ifdef BTS7960_MOTOR_DRIVER

  void initMotorController() {
    /* PWM frekansını ayarla: 20 kHz (default ~1 kHz yerine) */
    analogWriteFrequency(20000);

    pinMode(LEFT_MOTOR_FORWARD,   OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD,  OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD,  OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

    /* Başlangıçta tüm PWM çıkışlarını sıfırla — motorlar durur */
    analogWrite(LEFT_MOTOR_FORWARD,   0);
    analogWrite(LEFT_MOTOR_BACKWARD,  0);
    analogWrite(RIGHT_MOTOR_FORWARD,  0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd < 0) {
      spd = -spd;
      reverse = 1;
    }
    if (spd > MAX_PWM)
      spd = MAX_PWM;

    if (i == LEFT) {
      if (reverse == 0) {
        analogWrite(LEFT_MOTOR_FORWARD,  spd);
        analogWrite(LEFT_MOTOR_BACKWARD, 0);
      } else {
        analogWrite(LEFT_MOTOR_BACKWARD, spd);
        analogWrite(LEFT_MOTOR_FORWARD,  0);
      }
    } else {
      if (reverse == 0) {
        analogWrite(RIGHT_MOTOR_FORWARD,  spd);
        analogWrite(RIGHT_MOTOR_BACKWARD, 0);
      } else {
        analogWrite(RIGHT_MOTOR_BACKWARD, spd);
        analogWrite(RIGHT_MOTOR_FORWARD,  0);
      }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT,  leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#else
  #error A motor driver must be selected!
#endif

#endif  /* USE_BASE */
