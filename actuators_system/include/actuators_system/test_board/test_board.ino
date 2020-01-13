#include "Timer.h"
#include <Servo.h>

#define SERIAL_BAUDRATE 9600

#define PERIOD_IN_MS 20
#define INI_SERVO_PWM 1500

#define PIN_PWM_0   9
#define PIN_PWM_1   6
#define PIN_PWM_2   5
#define PIN_PWM_3  10
#define PIN_PWM_4  11

#define PIN_OUT_0  12

#define PIN_IN_0   A3
#define PIN_IN_1   A2
#define PIN_IN_2   A1
#define PIN_IN_3   A0

Timer timer = Timer(PERIOD_IN_MS);
uint16_t loop_count = 0;

Servo servo[5];
uint16_t pwm[5] = {INI_SERVO_PWM, INI_SERVO_PWM, INI_SERVO_PWM, INI_SERVO_PWM, INI_SERVO_PWM};
uint16_t pwm_min[5] = { 900,  900,  550,  900,  900};
uint16_t pwm_max[5] = {2100, 2100, 2400, 2100, 2100};

void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(5);
  while (!Serial) { ; }  // Wait for serial port to connect

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_OUT_0, OUTPUT);

  pinMode(PIN_IN_0, INPUT_PULLUP);
  pinMode(PIN_IN_1, INPUT);
  pinMode(PIN_IN_2, INPUT);
  pinMode(PIN_IN_3, INPUT);

  servo[0].attach(PIN_PWM_0, pwm_min[0], pwm_max[0]);
  servo[1].attach(PIN_PWM_1, pwm_min[1], pwm_max[1]);
  servo[2].attach(PIN_PWM_2, pwm_min[2], pwm_max[2]);
  servo[3].attach(PIN_PWM_3, pwm_min[3], pwm_max[3]);
  servo[4].attach(PIN_PWM_4, pwm_min[4], pwm_max[4]);

  timer.begin();
}

void loop() {

  bool any_digital_in = digitalRead(PIN_IN_0) && digitalRead(PIN_IN_1) && digitalRead(PIN_IN_2) && digitalRead(PIN_IN_3);
  digitalWrite(LED_BUILTIN, !any_digital_in);

  for (int i = 0; i < 5; i++) {
    if (loop_count < 100) {
      pwm[i] = pwm_max[i];
    } else if (loop_count < 200) {
      pwm[i] = pwm_min[i];
    } else {
      loop_count = 0;
    }    
    servo[i].writeMicroseconds(pwm[i]);
  }
  digitalWrite(PIN_OUT_0, !any_digital_in);

  loop_count++;
  timer.sleep();
}
