#include <Arduino.h>
#include "DCMotor.h"

// ── Hardware config ────────────────────────────────────────────────────────
// Left motor:  dir=GPIO13/12  enable=GPIO14  encoder=GPIO32/33  pwm_ch=0
// Right motor: dir=GPIO19/18  enable=GPIO21  encoder=GPIO22/23  pwm_ch=1  (dir pins swapped to fix inverted direction)
// L/R pin sets swapped (2026-10-25): turning was mirrored -- the GPIO19/18 motor is physically on the right.
DCMotor motorL(13, 12, 14, 32, 33, 0);
DCMotor motorR(19, 18, 21, 22, 23, 1);

// ── Encoder / kinematics ──────────────────────────────────────────────────
// 12V 333RPM gearmotor: 11 pulses/channel/motor-rev, x4 quadrature decoding
// (both edges of A and B) = 44 counts/motor-rev, x 1:30 gearbox = 1320.
// Verify by turning a wheel 10 revs by hand. Must match odom_node.py.
static constexpr float TICKS_PER_REV = 1320.0f;

// ── Motor model (333RPM motors, 12V pack, wheels free) ─────────────────────
// Breakaway: `sweep both 100 2` -> a stopped wheel starts at L≈98, R≈98 (+4).
static constexpr int PWM_BREAKAWAY_L = 102;
static constexpr int PWM_BREAKAWAY_R = 102;
// Once turning, friction is much lower (still spins at PWM 70). Steady-state
// PWM-vs-RPM fit over PWM 70..160: pwm ≈ offset + kf*rpm (max err ~4 PWM).
// L runs ~15-20 RPM faster than R at equal PWM, hence per-wheel offsets
// (values follow the physical motor -- swapped along with the pins above).
// Driver voltage drop means P:255 only reaches ~257 RPM.
static constexpr float FF_OFFSET_L = 39.0f;
static constexpr float FF_OFFSET_R = 51.0f;
static constexpr float FF_KF_L     = 0.70f;
static constexpr float FF_KF_R     = 0.71f;

// The feedforward offset assumes the wheel is already turning. A stopped
// wheel needs the higher breakaway PWM to get going, so bump up to it until
// it moves (only in the commanded direction; braking/reversing is untouched).
int applyBreakaway(int pwm, int breakaway, float currentRpm) {
  if (pwm == 0) return 0;
  if (fabsf(currentRpm) < 3.0f && abs(pwm) < breakaway) {
    return pwm > 0 ? breakaway : -breakaway;
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
  // Feedforward (offset + kf*target) gets close to the target immediately;
  // kp/ki only trim the residual and the extra load on the ground.
  float ffOffset;
  float kf;
  // Tuned in air (step test 30/60/120/-60 RPM): settles within ~1 RPM,
  // ~10% overshoot, t90 0.15-0.55 s. Re-check on the ground (expect more ki).
  float kp = 0.5f;
  float ki = 3.0f;
  float kd = 0.0f;
  float integral  = 0.0f;
  float prevError = 0.0f;
  // Gives up to ki*200 of sustained push -- headroom for ground load that
  // the in-air feedforward fit doesn't account for.
  static constexpr float INTEGRAL_LIMIT = 200.0f;
  // Errors smaller than this are treated as zero so the controller doesn't
  // dither on single-tick encoder quantisation (~0.9 RPM per tick at 20 Hz).
  static constexpr float ERROR_DEADBAND_RPM = 2.0f;

  PID(float ffOffset, float kf) : ffOffset(ffOffset), kf(kf) {}

  int compute(float target, float current, float dt) {
    float error = target - current;
    if (fabsf(error) < ERROR_DEADBAND_RPM) error = 0.0f;
    integral       = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float deriv    = (error - prevError) / dt;
    prevError      = error;
    float ff       = (target > 0.0f ? ffOffset : target < 0.0f ? -ffOffset : 0.0f)
                     + kf * target;
    float out      = ff + kp * error + ki * integral + kd * deriv;
    return constrain((int)out, -255, 255);
  }

  void reset() { integral = 0.0f; prevError = 0.0f; }
};

PID pidL(FF_OFFSET_L, FF_KF_L);
PID pidR(FF_OFFSET_R, FF_KF_R);

// Encoder counts saved between control ticks for RPM calculation
long prevTicksL = 0;
long prevTicksR = 0;

// ── Serial protocol ────────────────────────────────────────────────────────
// RX  "C:<left_rpm>:<right_rpm>\n"   e.g.  C:45.0:-20.5   (PID mode)
// RX  "P:<left_pwm>:<right_pwm>\n"  e.g.  P:40:40        (direct PWM, bypasses PID)
// RX  "K:<kp>:<ki>:<kd>[:<kf>]\n"   e.g.  K:0.3:1.5:0:0.7 (live gain update, both motors;
//                                   kf optional -- if omitted each wheel keeps its own)
// TX  "E:<left_ticks>:<right_ticks>\n"  e.g.  E:1234:-56
// TX  "K:ack:<kp>:<ki>:<kd>:<kfL>:<kfR>\n" acks a gain update

volatile bool directMode = false;
volatile int  directPwmL = 0;
volatile int  directPwmR = 0;

// The bridge re-sends C: at 20 Hz (same rate as the control loop), so
// resetting on every message would wipe the integral each tick. Only reset
// when starting from stop or reversing direction.
static bool needsPidReset(float oldTarget, float newTarget) {
  return oldTarget == 0.0f || newTarget == 0.0f ||
         (oldTarget > 0.0f) != (newTarget > 0.0f);
}

void parseCommand(const String& line) {
  if (line.startsWith("C:")) {
    int sep = line.indexOf(':', 2);
    if (sep < 0) return;
    float l = line.substring(2, sep).toFloat();
    float r = line.substring(sep + 1).toFloat();
    noInterrupts();
    bool  wasDirect = directMode;
    float oldL      = targetRpmL;
    float oldR      = targetRpmR;
    directMode = false;
    targetRpmL = l;
    targetRpmR = r;
    interrupts();
    if (wasDirect || needsPidReset(oldL, l)) pidL.reset();
    if (wasDirect || needsPidReset(oldR, r)) pidR.reset();

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
    int sep3 = line.indexOf(':', sep2 + 1);
    float kp = line.substring(2, sep1).toFloat();
    float ki = line.substring(sep1 + 1, sep2).toFloat();
    float kd = line.substring(sep2 + 1, sep3 < 0 ? line.length() : sep3).toFloat();

    pidL.kp = kp; pidL.ki = ki; pidL.kd = kd;
    pidR.kp = kp; pidR.ki = ki; pidR.kd = kd;
    if (sep3 >= 0) {
      float kf = line.substring(sep3 + 1).toFloat();
      pidL.kf = kf; pidR.kf = kf;
    }
    pidL.reset();
    pidR.reset();

    Serial.print("K:ack:");
    Serial.print(kp, 3);
    Serial.print(":");
    Serial.print(ki, 3);
    Serial.print(":");
    Serial.print(kd, 3);
    Serial.print(":");
    Serial.print(pidL.kf, 3);
    Serial.print(":");
    Serial.println(pidR.kf, 3);
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
    motorL.setSpeed(applyBreakaway(pidL.compute(tL, currentRpmL, dt), PWM_BREAKAWAY_L, currentRpmL));
    motorR.setSpeed(applyBreakaway(pidR.compute(tR, currentRpmR, dt), PWM_BREAKAWAY_R, currentRpmR));
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
