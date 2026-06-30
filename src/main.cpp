#include <Arduino.h>
#include "DCMotor.h"

// ── Hardware config ────────────────────────────────────────────────────────
// Left motor:  dir=GPIO19/18  enable=GPIO21  encoder=GPIO22/23  pwm_ch=0  (dir pins swapped to fix inverted direction)
// Right motor: dir=GPIO13/12  enable=GPIO14  encoder=GPIO32/33  pwm_ch=1
DCMotor motorL(19, 18, 21, 22, 23, 0);
DCMotor motorR(13, 12, 14, 32, 33, 1);

// ── Encoder / kinematics placeholders ─────────────────────────────────────
// TODO: set to (encoder CPR) × (gear ratio) for your motor
static constexpr float TICKS_PER_REV = 1440.0f;

// ── Dead-band compensation ─────────────────────────────────────────────────
// Measured via `sweep` in arduino_pi_serial.py with motors cold: L≈138, R≈144.
// Batteries are weak right now (low pack voltage -> low torque at low PWM),
// so the real stiction point is much higher than a healthy 30. +4 margin.
static constexpr int PWM_DEADBAND_L = 142;
static constexpr int PWM_DEADBAND_R = 148;

// Pushes a PID output past the motor's dead-band -- but only when the
// commanded direction agrees with (or the motor is near-stopped relative
// to) the current direction of motion. On this weak-battery rig, sustaining
// motion takes nearly as much PWM as starting it, so the boost must stay on
// while accelerating/holding in the same direction. It only needs to drop
// out when the PID is asking to brake/reverse against current motion --
// inflating a small braking correction by the full deadband would turn a
// gentle trim into a near-full-power reversal (the dip seen while tuning).
int applyDeadband(int pwm, int deadband, float currentRpm) {
  if (pwm == 0) return 0;
  bool sameDirection = (pwm > 0 && currentRpm > -3.0f) ||
                       (pwm < 0 && currentRpm <  3.0f);
  if (sameDirection) {
    return pwm > 0 ? constrain(pwm + deadband, 0, 255)
                   : constrain(pwm - deadband, -255, 0);
  }
  return constrain(pwm, -255, 255);
}

// ── Control loop timing ────────────────────────────────────────────────────
static constexpr uint32_t CONTROL_INTERVAL_MS = 50;   // 20 Hz
static uint32_t lastControlMs = 0;

// ── Target RPM (written by serial parser, read by control loop) ────────────
volatile float targetRpmL = 0.0f;
volatile float targetRpmR = 0.0f;

// ── PID state ─────────────────────────────────────────────────────────────
struct PID {
  // Scaled down vs the old kp=1.5: with deadband ~142-148, the proportional
  // term only has ~110 PWM of headroom left (255-145) instead of ~225, so the
  // same kp would snap straight to near-full power on small errors.
  float kp = 0.886f;   // TODO: tune
  float ki = 0.592f;  // lowered slightly; deadband kick already does most of the work
  float kd = 0.013f;  // raised a bit for extra damping against the bigger PWM jump
  float integral  = 0.0f;
  float prevError = 0.0f;
  // Was 50 -- too tight. With deadband ~145, ki needs real headroom to
  // supply enough sustained torque at higher RPM (weak battery -> more sag
  // under load), or steady-state error never closes no matter how high kp
  // goes. 200 gives up to ki*200 of sustained push instead of ki*50.
  static constexpr float INTEGRAL_LIMIT = 200.0f;
  // Errors smaller than this are treated as zero so the controller doesn't
  // dither right at the PWM deadband boundary (commanding ~145 then 0 then ~145...).
  static constexpr float ERROR_DEADBAND_RPM = 2.0f;

  int compute(float target, float current, float dt) {
    float error = target - current;
    if (fabsf(error) < ERROR_DEADBAND_RPM) error = 0.0f;
    integral       = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float deriv    = (error - prevError) / dt;
    prevError      = error;
    float out      = kp * error + ki * integral + kd * deriv;
    return constrain((int)out, -255, 255);
  }

  void reset() { integral = 0.0f; prevError = 0.0f; }
};

PID pidL, pidR;

// Encoder counts saved between control ticks for RPM calculation
long prevTicksL = 0;
long prevTicksR = 0;

// ── Serial protocol ────────────────────────────────────────────────────────
// RX  "C:<left_rpm>:<right_rpm>\n"   e.g.  C:45.0:-20.5   (PID mode)
// RX  "P:<left_pwm>:<right_pwm>\n"  e.g.  P:40:40        (direct PWM, bypasses PID)
// RX  "K:<kp>:<ki>:<kd>\n"          e.g.  K:0.8:0.08:0.08 (live PID gain update, both motors)
// TX  "E:<left_ticks>:<right_ticks>\n"  e.g.  E:1234:-56
// TX  "K:ack:<kp>:<ki>:<kd>\n"      acks a gain update

volatile bool directMode = false;
volatile int  directPwmL = 0;
volatile int  directPwmR = 0;

void parseCommand(const String& line) {
  if (line.startsWith("C:")) {
    int sep = line.indexOf(':', 2);
    if (sep < 0) return;
    float l = line.substring(2, sep).toFloat();
    float r = line.substring(sep + 1).toFloat();
    noInterrupts();
    directMode = false;
    targetRpmL = l;
    targetRpmR = r;
    interrupts();
    pidL.reset();
    pidR.reset();

  } else if (line.startsWith("P:")) {
    int sep = line.indexOf(':', 2);
    if (sep < 0) return;
    int l = line.substring(2, sep).toInt();
    int r = line.substring(sep + 1).toInt();
    noInterrupts();
    directMode = true;
    directPwmL = l;
    directPwmR = r;
    interrupts();

  } else if (line.startsWith("K:")) {
    int sep1 = line.indexOf(':', 2);
    if (sep1 < 0) return;
    int sep2 = line.indexOf(':', sep1 + 1);
    if (sep2 < 0) return;
    float kp = line.substring(2, sep1).toFloat();
    float ki = line.substring(sep1 + 1, sep2).toFloat();
    float kd = line.substring(sep2 + 1).toFloat();

    pidL.kp = kp; pidL.ki = ki; pidL.kd = kd;
    pidR.kp = kp; pidR.ki = ki; pidR.kd = kd;
    pidL.reset();
    pidR.reset();

    Serial.print("K:ack:");
    Serial.print(kp, 3);
    Serial.print(":");
    Serial.print(ki, 3);
    Serial.print(":");
    Serial.println(kd, 3);
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  motorL.begin();
  motorR.begin();

  prevTicksL = motorL.getEncoderCount();
  prevTicksR = motorR.getEncoderCount();

  Serial.println("ESP32 ready. Protocol: RX=C:<lRPM>:<rRPM>  TX=E:<lTicks>:<rTicks>");
}

// ── Main loop ─────────────────────────────────────────────────────────────
void loop() {
  // ── Parse incoming commands ──
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) parseCommand(line);
  }

  // ── 20 Hz control + feedback loop ──
  uint32_t now = millis();
  if (now - lastControlMs < CONTROL_INTERVAL_MS) return;
  float dt = (now - lastControlMs) / 1000.0f;
  lastControlMs = now;

  // Measure RPM from encoder delta
  long ticksL = motorL.getEncoderCount();
  long ticksR = motorR.getEncoderCount();

  float deltaL = ticksL - prevTicksL;
  float deltaR = ticksR - prevTicksR;
  prevTicksL   = ticksL;
  prevTicksR   = ticksR;

  float currentRpmL = (deltaL / TICKS_PER_REV) * (60.0f / dt);
  float currentRpmR = (deltaR / TICKS_PER_REV) * (60.0f / dt);

  // Read targets / mode atomically
  noInterrupts();
  bool dm  = directMode;
  int  dL  = directPwmL;
  int  dR  = directPwmR;
  float tL = targetRpmL;
  float tR = targetRpmR;
  interrupts();

  if (dm) {
    // Direct PWM mode: bypass PID entirely (used for deadband sweep)
    motorL.setSpeed(dL);
    motorR.setSpeed(dR);
  } else if (tL == 0.0f && tR == 0.0f) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    pidL.reset();
    pidR.reset();
  } else {
    motorL.setSpeed(applyDeadband(pidL.compute(tL, currentRpmL, dt), PWM_DEADBAND_L, currentRpmL));
    motorR.setSpeed(applyDeadband(pidR.compute(tR, currentRpmR, dt), PWM_DEADBAND_R, currentRpmR));
  }

  // Send encoder feedback to ROS2 (keep format strict for parser)
  Serial.print("E:");
  Serial.print(ticksL);
  Serial.print(":");
  Serial.println(ticksR);

  // Debug line: current RPM (visible in [RAW] log, ignored by parser)
  Serial.print("D:rpm=");
  Serial.print(currentRpmL, 1);
  Serial.print(":");
  Serial.println(currentRpmR, 1);
}
