/**
 * Motor control with speed and position PID control using quadrature encoder feedback.
 * こちらで公開されている Nidec 24HBLDC用の QDDギヤを使用し、速度・位置制御を行うサンプルコードです。
 * https://suzu-mono-gram.com/blog/nidec-24h-motor-guide/
 */

#include <Arduino.h>
#include "Nidec24HBLDC_QDD.hpp"

// 暴走回転するときは、A相・B相の配線を確認する

// Motor1 pins
#define PIN_PWM 1
#define PIN_BRAKE 2
#define PIN_DIR 3
#define PIN_ENCODER_A 4
#define PIN_ENCODER_B 5

// Motor2 pins
#define PIN_PWM2 6
#define PIN_BRAKE2 7
#define PIN_DIR2 8
#define PIN_ENCODER_A2 9
#define PIN_ENCODER_B2 10

// モーターを2個使用するデモとする場合は、以下の行のコメントアウトを外す
// #define USE_2nd_MOTOR

Nidec24HBLDC_QDD motor1(PIN_PWM, PIN_BRAKE, PIN_DIR, PIN_ENCODER_A, PIN_ENCODER_B); // 動く
#ifdef USE_2nd_MOTOR
Nidec24HBLDC_QDD motor2(PIN_PWM2, PIN_BRAKE2, PIN_DIR2, PIN_ENCODER_A2, PIN_ENCODER_B2);
#endif

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  motor1.begin();
#ifdef USE_2nd_MOTOR
  motor2.begin();
#endif
}

void loop() {
  motor1.update();
#ifdef USE_2nd_MOTOR
  motor2.update();
#endif

  {
    uint32_t current_time = millis();
    static uint32_t last_time = 0;
    static uint32_t angle = 0;
    static bool direction = true;
    if (current_time - last_time >= 2000) {
      if (direction) {
        motor1.setAngle(0);
      } else {
        motor1.setAngle(180);
      }
      direction = !direction;
      last_time = current_time;
    }
  }

#ifdef USE_2nd_MOTOR
  {
    static int speed = 0;
    static uint32_t last_time = 0;
    static bool speed_up = true;
    uint32_t current_time = millis();
    if (current_time - last_time >= 10){
      motor2.setSpeed(speed);
      if(speed_up){
        speed += 2;
      } else {
        speed -= 2;
      }
      if (abs(speed) > 600) {
        speed_up = false;
      } else if (abs(speed) < 10) {
        speed_up = true;
      } 
      last_time = current_time;
    }
  }
#endif
}
