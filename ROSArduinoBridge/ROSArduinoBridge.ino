/*********************************************************************
 *  ROSArduinoBridge — STM32F407G-DISC1 versiyonu
 *
 *  Hedef kart : STM32F407G-DISC1
 *  Arduino IDE board seçimi:
 *    Board Manager  : STM32 MCU based boards (stm32duino)
 *    Board          : Generic STM32F4 series
 *    Board part     : Discovery F407VG
 *    USB support    : CDC (generic 'Serial' supersede U(S)ART)
 *    Upload method  : STM32CubeProgrammer (SWD)
 *
 *  Serial: USB CDC (micro-USB USER soketi, PA11/PA12)
 *  NOT: STM32 seri port açılınca Arduino Mega'nın aksine reset ATMAZ.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code

#ifdef USE_BASE
   #define STM32_HW_ENCODER    // STM32 donanım encoder modu (TIM2/TIM3)
   #define BTS7960_MOTOR_DRIVER
#endif

#undef USE_SERVOS

/* Serial port baud rate — USB CDC'de görmezden gelinir ama protokol uyumu için tutulur */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

/* Buzzer pin — PE7 (STM32F407G-DISC1 üzerinde serbest pin) */
#define BUZZER_PIN     PE7

/* Buzzer pattern configuration */
#define BEEP_ON_TIME   200
#define BEEP_OFF_TIME  300

#include "Arduino.h"
#include "commands.h"
#include "sensors.h"

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"

  /* PID döngüsü: 30 Hz — ROS2 loop_rate parametresi ile eşleşmeli */
  #define PID_RATE       30
  const int PID_INTERVAL = 1000 / PID_RATE;
  unsigned long nextPID  = 0;

  #define AUTO_STOP_INTERVAL 2000
  unsigned long lastMotorCommand = 0;
#endif

int arg   = 0;
int idx   = 0;  /* "index" ARM strings.h'da char* index(const char*,int) olarak tanımlı — çakışmayı önle */
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

bool buzzer_enabled   = false;
bool buzzer_state     = false;
unsigned long last_beep_time = 0;

void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg  = 0;
  idx  = 0;
}

void updateBuzzer() {
  if (!buzzer_enabled) {
    if (buzzer_state) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state = false;
    }
    return;
  }
  unsigned long t = millis();
  if (buzzer_state) {
    if (t - last_beep_time >= BEEP_ON_TIME) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state     = false;
      last_beep_time   = t;
    }
  } else {
    if (t - last_beep_time >= BEEP_OFF_TIME) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzer_state     = true;
      last_beep_time   = t;
    }
  }
}

void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  char *saveptr;
  int pid_args[4] = {Kp, Kd, Ki, Ko};  /* Mevcut değerlerle başlat — eksik argümanda bozulma olmaz */
  arg1 = atol(argv1);
  arg2 = atol(argv2);

  switch (cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case BUZZER_CONTROL:
    buzzer_enabled = (arg1 == 1);
    if (!buzzer_enabled) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state = false;
    }
    Serial.println("OK");
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case DIGITAL_WRITE:
    if      (arg2 == 0) { digitalWrite(arg1, LOW);    Serial.println("OK"); }
    else if (arg2 == 1) { digitalWrite(arg1, HIGH);   Serial.println("OK"); }
    else                  Serial.println("Invalid Command");
    break;
  case PIN_MODE:
    if      (arg2 == 0) { pinMode(arg1, INPUT);        Serial.println("OK"); }
    else if (arg2 == 1) { pinMode(arg1, OUTPUT);       Serial.println("OK"); }
    else                  Serial.println("Invalid Command");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;

#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    } else {
      moving = 1;
    }
    leftPID.TargetTicksPerFrame  = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;
  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    resetPID();
    moving = 0;
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK");
    break;
  case UPDATE_PID:
    for (str = strtok_r(p, ":", &saveptr); str != NULL; str = strtok_r(NULL, ":", &saveptr)) {
      if (i < 4) pid_args[i++] = atoi(str);
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif

  default:
    Serial.println("Invalid Command");
    break;
  }
}

void setup() {
  /* USB CDC — baud rate parametresi sembolik, gerçek hız USB tarafından belirlenir */
  Serial.begin(BAUDRATE);

  /* Buzzer */
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer_enabled = false;
  buzzer_state   = false;
  last_beep_time = millis();

#ifdef USE_BASE
  /* Donanım encoder timer'larını başlat (TIM2 sol, TIM3 sağ) */
  initEncoders();

  /* Motor sürücüyü başlat */
  initMotorController();

  /* PID'yi sıfırla */
  resetPID();
#endif
}

void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    if (chr == 13) {  // CR ile komut biter
      if (arg == 1)      argv1[idx] = '\0';
      else if (arg == 2) argv2[idx] = '\0';
      runCommand();
      resetCommand();
    } else if (chr == ' ') {
      if (arg == 0) {
        arg = 1;
      } else if (arg == 1) {
        argv1[idx] = '\0';
        arg = 2;
        idx = 0;
      } else if (arg == 2) {
        /* Fazladan boşluk: argv2'yi sonlandır, daha fazla karakter kabul etme */
        argv2[idx] = '\0';
        idx = 15;
      }
      continue;
    } else {
      if (arg == 0)      cmd = chr;
      else if (arg == 1) { if (idx < 15) argv1[idx++] = chr; }
      else if (arg == 2) { if (idx < 15) argv2[idx++] = chr; }
    }
  }

  updateBuzzer();

#ifdef USE_BASE
  if ((millis() - nextPID) >= (unsigned long)PID_INTERVAL) {
    updatePID();
    nextPID = millis();  /* Resync — gecikme sonrası catch-up döngüsünü önle */
  }
  if (moving && (millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif
}
