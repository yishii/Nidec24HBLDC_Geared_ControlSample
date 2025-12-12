#include "Nidec24HBLDC_QDD.hpp"

#include <cmath>

namespace {
constexpr int kPpr = 100;
constexpr float kGearRatio = 10.0f;  // gear ratio 10:1
constexpr int kCountsPerRev = static_cast<int>(kPpr * 4 * kGearRatio);
constexpr uint32_t kControlIntervalMs = 10;  // control period [ms]
constexpr int kBrakeOn = LOW;
constexpr int kBrakeOff = HIGH;
}  // namespace

Nidec24HBLDC_QDD* Nidec24HBLDC_QDD::instances_[kMaxInstances] = {nullptr};

Nidec24HBLDC_QDD::Nidec24HBLDC_QDD(int pin_pwm, int pin_brake, int pin_dir,
                                   int pin_enc_a, int pin_enc_b)
    : pin_pwm_(pin_pwm),
      pin_brake_(pin_brake),
      pin_dir_(pin_dir),
      pin_enc_a_(pin_enc_a),
      pin_enc_b_(pin_enc_b) {}

void Nidec24HBLDC_QDD::begin() {
  pinMode(pin_pwm_, OUTPUT);
  pinMode(pin_brake_, OUTPUT);
  pinMode(pin_dir_, OUTPUT);
  pinMode(pin_enc_a_, INPUT_PULLUP);
  pinMode(pin_enc_b_, INPUT_PULLUP);

  analogWriteFreq(20000);   // 20 kHz
  analogWriteRange(255);    // 8bitレンジ
  drivePwm(0);

  if (instance_index_ == kInvalidInstance) {
    for (uint8_t i = 0; i < kMaxInstances; ++i) {
      if (instances_[i] == nullptr) {
        instances_[i] = this;
        instance_index_ = i;
        break;
      }
    }
  }
  if (instance_index_ == kInvalidInstance) {
    Serial.println("Nidec24HBLDC_QDD: max instance count reached");
    return;
  }

  encoder_count_ = 0;
  last_encoded_ = (digitalRead(pin_enc_a_) << 1) | digitalRead(pin_enc_b_);

  static void (*const kHandleA[kMaxInstances])() = {
      handleEncoderA0, handleEncoderA1, handleEncoderA2, handleEncoderA3};
  static void (*const kHandleB[kMaxInstances])() = {
      handleEncoderB0, handleEncoderB1, handleEncoderB2, handleEncoderB3};
  attachInterrupt(digitalPinToInterrupt(pin_enc_a_),
                  kHandleA[instance_index_], CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_enc_b_),
                  kHandleB[instance_index_], CHANGE);

  last_update_ms_ = millis();
  last_sample_count_ = 0;
  last_report_ms_ = millis();
  last_reported_angle_deg_ = 0.0f;
}

void Nidec24HBLDC_QDD::setPwm(int pwm) {
  mode_ = Mode::kManual;
  integral_speed_ = 0;
  integral_position_ = 0;
  current_pwm_ = constrain(pwm, -255, 255);
  drivePwm(current_pwm_);
}

void Nidec24HBLDC_QDD::setSpeed(float rpm) {
  target_rpm_ = rpm;
  integral_speed_ = 0;
  mode_ = Mode::kSpeed;
}

void Nidec24HBLDC_QDD::setAngle(float degree) {
  target_counts_ = static_cast<long>((degree / 360.0f) * kCountsPerRev);
  integral_position_ = 0;
  mode_ = Mode::kPosition;
}

long Nidec24HBLDC_QDD::encoderCount() const { return encoder_count_; }

void Nidec24HBLDC_QDD::update() {
  const uint32_t now = millis();
  const uint32_t dt = now - last_update_ms_;
  if (dt < kControlIntervalMs) return;
  last_update_ms_ = now;

  const long count_snapshot = encoder_count_;
  const long delta = count_snapshot - last_sample_count_;
  last_sample_count_ = count_snapshot;

  // 現在の速度[rpm]
  const float measured_rpm =
      (delta * 60000.0f) / (static_cast<float>(kCountsPerRev) * dt);
  last_measured_rpm_ = measured_rpm;

  int pwm_cmd = current_pwm_;

  switch (mode_) {
    case Mode::kSpeed: {
      constexpr float kP = 1.2f;
      constexpr float kI = 0.4f;
      constexpr float kIClamp = 150.0f;

      const float err = target_rpm_ - measured_rpm;
      integral_speed_ += err * (dt / 1000.0f);
      integral_speed_ =
          constrain(integral_speed_, -kIClamp, kIClamp);  // windup防止
      const float out = kP * err + kI * integral_speed_;
      pwm_cmd = constrain(static_cast<int>(round(out)), -255, 255);
      break;
    }
    case Mode::kPosition: {
      constexpr float kPpos = 1.0f;
      constexpr float kIpos = 0.02f;
      constexpr float kIposClamp = 200.0f;
      constexpr int hold_deadband_counts = 10;  // 小さな揺れを抑える
      const long err_counts = target_counts_ - count_snapshot;
      if (abs(err_counts) <= hold_deadband_counts) {
        integral_position_ = 0;  // 近傍での押し付け過多を防ぐ
      }
      const long signed_err =
          (abs(err_counts) <= hold_deadband_counts) ? 0 : err_counts;
      integral_position_ += signed_err * (dt / 1000.0f);
      integral_position_ =
          constrain(integral_position_, -kIposClamp, kIposClamp);
      const float out =
          kPpos * signed_err + kIpos * integral_position_;  // PI位置制御
      pwm_cmd = constrain(static_cast<int>(round(out)), -255, 255);

      // 追加 : 静止摩擦を考慮した最低保持PWM値
      constexpr int kMinHoldPwm = 20;  // 調整 : モーター停止時にブルブルする場合はここの数値を大きくする (しょっちゅういじる場合はパラメータをライブラリの外に出そう)
      if (pwm_cmd > 0 && pwm_cmd < kMinHoldPwm) pwm_cmd = kMinHoldPwm;
      if (pwm_cmd < 0 && pwm_cmd > -kMinHoldPwm) pwm_cmd = -kMinHoldPwm;
      break;
    }
    case Mode::kManual:
    default:
      pwm_cmd = current_pwm_;
      break;
  }

  current_pwm_ = pwm_cmd;
  drivePwm(pwm_cmd);

  // ログ出力: 速度モードは一定周期、角度モードは1度以上変化で通知
  if (mode_ == Mode::kSpeed) {
    constexpr uint32_t kReportIntervalMs = 200;
    if (now - last_report_ms_ >= kReportIntervalMs) {
      Serial.print("rpm: ");
      Serial.println(last_measured_rpm_, 2);
      last_report_ms_ = now;
    }
  } else if (mode_ == Mode::kPosition) {
    const float angle_deg =
        (static_cast<float>(count_snapshot) * 360.0f) / kCountsPerRev;
    if (fabs(angle_deg - last_reported_angle_deg_) >= 1.0f) {
      Serial.print("deg: ");
      Serial.println(angle_deg, 2);
      last_reported_angle_deg_ = angle_deg;
      last_report_ms_ = now;
    }
  }
}

void Nidec24HBLDC_QDD::dispatchEncoder(uint8_t idx) {
  if (idx < kMaxInstances && instances_[idx]) {
    instances_[idx]->updateEncoder();
  }
}

void Nidec24HBLDC_QDD::handleEncoderA0() { dispatchEncoder(0); }
void Nidec24HBLDC_QDD::handleEncoderA1() { dispatchEncoder(1); }
void Nidec24HBLDC_QDD::handleEncoderA2() { dispatchEncoder(2); }
void Nidec24HBLDC_QDD::handleEncoderA3() { dispatchEncoder(3); }
void Nidec24HBLDC_QDD::handleEncoderB0() { dispatchEncoder(0); }
void Nidec24HBLDC_QDD::handleEncoderB1() { dispatchEncoder(1); }
void Nidec24HBLDC_QDD::handleEncoderB2() { dispatchEncoder(2); }
void Nidec24HBLDC_QDD::handleEncoderB3() { dispatchEncoder(3); }

// 割り込み側のクアドラチャ処理は最短限にしてCPU負荷を抑える
void Nidec24HBLDC_QDD::updateEncoder() {
  const int a = digitalRead(pin_enc_a_);
  const int b = digitalRead(pin_enc_b_);
  const int encoded = (a << 1) | b;
  const int sum = (last_encoded_ << 2) | encoded;

  // 00->01->11->10->00 が正転
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoder_count_++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 ||
             sum == 0b1000) {
    encoder_count_--;
  }
  last_encoded_ = encoded;
}

void Nidec24HBLDC_QDD::drivePwm(int pwm) {
  const int clamped = constrain(pwm, -255, 255);
  // Flip sign if encoder counts reverse relative to motor
  constexpr int kMotorDirSign = -1;
  const int signed_pwm = clamped * kMotorDirSign;

  if (signed_pwm != 0) {
    const int abs_pwm = abs(signed_pwm);
    // Inverted logic: analogWrite 255 -> 0% duty, 0 -> 100% duty
    // Clamp to avoid hitting the stop duty (255) while commanding motion
    const int duty = constrain(255 - abs_pwm, 0, 240);

    digitalWrite(PIN_LED, LOW);
    digitalWrite(pin_dir_, (signed_pwm > 0) ? HIGH : LOW);
    digitalWrite(pin_brake_, kBrakeOff);  // release brake while driving
    analogWrite(pin_pwm_, duty);
  } else {
    // Stop: engage active-low brake and command 0% duty (value 255)
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(pin_brake_, kBrakeOn);
    analogWrite(pin_pwm_, 255);
  }
}
