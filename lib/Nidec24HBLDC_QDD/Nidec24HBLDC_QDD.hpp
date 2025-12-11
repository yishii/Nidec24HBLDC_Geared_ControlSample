#pragma once

#include <Arduino.h>

class Nidec24HBLDC_QDD {
 public:
  Nidec24HBLDC_QDD(int pin_pwm, int pin_brake, int pin_dir, int pin_enc_a,
                   int pin_enc_b);

  void begin();
  void setPwm(int pwm);
  void setSpeed(float rpm);
  void setAngle(float degree);
  long encoderCount() const;
  void update();

 private:
  enum class Mode { kManual, kSpeed, kPosition };

  static void handleEncoderA();
  static void handleEncoderB();
  void updateEncoder();
  void drivePwm(int pwm);

  int pin_pwm_;
  int pin_brake_;
  int pin_dir_;
  int pin_enc_a_;
  int pin_enc_b_;

  Mode mode_ = Mode::kManual;
  float target_rpm_ = 0.0f;
  long target_counts_ = 0;
  float integral_speed_ = 0.0f;
  float integral_position_ = 0.0f;
  int current_pwm_ = 0;
  float last_measured_rpm_ = 0.0f;

  uint32_t last_update_ms_ = 0;
  long last_sample_count_ = 0;
  uint32_t last_report_ms_ = 0;
  float last_reported_angle_deg_ = 0.0f;

  static Nidec24HBLDC_QDD* instance_;
  static volatile long encoder_count_;
  static volatile int last_encoded_;
};
