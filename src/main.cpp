/**
 * Motor control with speed and position PID control using quadrature encoder feedback.
 * こちらで公開されているモーター・ギヤボックスで速度・位置制御を行うサンプルコードです。
 * https://suzu-mono-gram.com/blog/nidec-24h-motor-guide/
 */

#include <Arduino.h>
#include <cmath>

#define PIN_PWM 1
#define PIN_BRAKE 2
#define PIN_DIR 3
/* Encoderは一周100パルス(PPR) */
#define PIN_ENCODER_A 4
#define PIN_ENCODER_B 5

#define BRAKE_ON LOW
#define BRAKE_OFF HIGH

constexpr int kPpr = 100;
constexpr float kGearRatio = 10.0f;  // gear ratio 10:1
constexpr int kCountsPerRev = static_cast<int>(kPpr * 4 * kGearRatio);
constexpr uint32_t kControlIntervalMs = 10;      // control period [ms]
class Motor {
 public:
  Motor(int pin_pwm, int pin_brake, int pin_dir, int pin_enc_a, int pin_enc_b)
      : pin_pwm_(pin_pwm),
        pin_brake_(pin_brake),
        pin_dir_(pin_dir),
        pin_enc_a_(pin_enc_a),
        pin_enc_b_(pin_enc_b) {}

  void begin() {
    pinMode(pin_pwm_, OUTPUT);
    pinMode(pin_brake_, OUTPUT);
    pinMode(pin_dir_, OUTPUT);
    pinMode(pin_enc_a_, INPUT_PULLUP);
    pinMode(pin_enc_b_, INPUT_PULLUP);

    analogWriteFreq(20000);   // 20 kHz
    analogWriteRange(255);    // 8bitレンジ
    drivePwm(0);

    instance_ = this;
    encoder_count_ = 0;
    last_encoded_ = (digitalRead(pin_enc_a_) << 1) | digitalRead(pin_enc_b_);
    attachInterrupt(digitalPinToInterrupt(pin_enc_a_), handleEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_enc_b_), handleEncoderB, CHANGE);

    last_update_ms_ = millis();
    last_sample_count_ = 0;
    last_report_ms_ = millis();
    last_reported_angle_deg_ = 0.0f;
  }

  void setPwm(int pwm) {
    mode_ = Mode::kManual;
    integral_speed_ = 0;
    integral_position_ = 0;
    current_pwm_ = constrain(pwm, -255, 255);
    drivePwm(current_pwm_);
  }

  void setSpeed(float rpm) {
    target_rpm_ = rpm;
    integral_speed_ = 0;
    mode_ = Mode::kSpeed;
  }

  void setAngle(float degree) {
    target_counts_ = static_cast<long>((degree / 360.0f) * kCountsPerRev);
    integral_position_ = 0;
    mode_ = Mode::kPosition;
  }

  long encoderCount() const { return encoder_count_; }

  void update() {
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
        constexpr int hold_deadband_counts = 10;  // 小さな揺れを抑えめE
        const long err_counts = target_counts_ - count_snapshot;
        if (abs(err_counts) <= hold_deadband_counts) {
          integral_position_ = 0; // 近傍での押し付け蓄積を防ぐ
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
        constexpr int kMinHoldPwm = 20;  // 20等だとブルブルする場合がある
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

    // ログ出劁E 速度モード�E一定周期、角度モード�E1度以上変化で通知
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

 private:
  enum class Mode { kManual, kSpeed, kPosition };

  static void handleEncoderA() {
    if (instance_) instance_->updateEncoder();
  }
  static void handleEncoderB() {
    if (instance_) instance_->updateEncoder();
  }

  // 割り込み冁E��のクアドラチャ処琁E�E最短限にしてCPU負荷を抑える
  void updateEncoder() {
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

  void drivePwm(int pwm) {
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
      digitalWrite(pin_brake_, BRAKE_OFF);  // release brake while driving
      analogWrite(pin_pwm_, duty);
    } else {
      // Stop: engage active-low brake and command 0% duty (value 255)
      digitalWrite(PIN_LED, HIGH);
      digitalWrite(pin_brake_, BRAKE_ON);
      analogWrite(pin_pwm_, 255);
    }
  }

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

  static inline Motor* instance_ = nullptr;
  static inline volatile long encoder_count_ = 0;
  static inline volatile int last_encoded_ = 0;
};

Motor motor1(PIN_PWM, PIN_BRAKE, PIN_DIR, PIN_ENCODER_A, PIN_ENCODER_B);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  motor1.begin();

  // 侁E 初期位置で制御開姁E  motor1.setAngle(0.0f);
}

void loop() {
  motor1.update();  // 制御周期でPWMを更新

  uint32_t current_time = millis();
  static uint32_t last_time = 0;
  static uint32_t angle = 0;
  static bool direction = true;
  if (current_time - last_time >= 2000) {
    // angle += 10;
    // motor1.setAngle(angle % 360);
    if(direction){
      motor1.setAngle(0);
    }else{
      motor1.setAngle(180);
    }
    direction = !direction;
    last_time = current_time;
  }

  // // チE��用: 一定時間ごとに速度制御→角度制御を�Eり替える
  // static uint32_t last_switch = millis();
  // static int state = 0;
  // const uint32_t now = millis();

  // if (state == 0 && now - last_switch > 2000) {
  //   motor1.setSpeed(60.0f);  // 60rpmで回転
  //   state = 1;
  //   last_switch = now;
  // } else if (state == 1 && now - last_switch > 4000) {
  //   motor1.setAngle(180.0f);  // 180度位置決めE  //   state = 2;
  //   last_switch = now;
  // } else if (state == 2 && now - last_switch > 4000) {
  //   motor1.setAngle(0.0f);  // 基準位置に戻めE外力で動いても戻ぁE
  //   state = 0;
  //   last_switch = now;
  // }
}
