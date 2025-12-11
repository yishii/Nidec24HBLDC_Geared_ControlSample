/**
 * Motor control with speed and position PID control using quadrature encoder feedback.
 * こちらで公開されている Nidec 24HBLDC用の QDDギヤを使用し、速度・位置制御を行うサンプルコードです。
 * https://suzu-mono-gram.com/blog/nidec-24h-motor-guide/
 */

#include <Arduino.h>
#include "Nidec24HBLDC_QDD.hpp"

// モーターのピン定義
#define PIN_PWM 1
#define PIN_BRAKE 2
#define PIN_DIR 3
#define PIN_ENCODER_A 4
#define PIN_ENCODER_B 5

Nidec24HBLDC_QDD motor1(PIN_PWM, PIN_BRAKE, PIN_DIR, PIN_ENCODER_A, PIN_ENCODER_B);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  motor1.begin();
}

void loop() {
  motor1.update();  // モーター制御処理を定期的に呼び出す

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
}
