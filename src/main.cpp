#include <Arduino.h>
#include "DCMotor.h"

// ── Hardware config ────────────────────────────────────────────────────────
// Left motor:  dir=GPIO18/19  enable=GPIO21  encoder=GPIO22/23  pwm_ch=0
// Right motor: dir=GPIO25/26  enable=GPIO27  encoder=GPIO32/33  pwm_ch=1
DCMotor motorL(18, 19, 21, 22, 23, 0);
DCMotor motorR(25, 26, 27, 32, 33, 1);

// ── Encoder / kinematics placeholders ─────────────────────────────────────
// TODO: set to (encoder CPR) × (gear ratio) for your motor
static constexpr float TICKS_PER_REV = 1440.0f;

// ── Control loop timing ────────────────────────────────────────────────────
static constexpr uint32_t CONTROL_INTERVAL_MS = 50;   // 20 Hz
static uint32_t lastControlMs = 0;

// ── Target RPM (written by serial parser, read by control loop) ────────────
volatile float targetRpmL = 0.0f;
volatile float targetRpmR = 0.0f;

// ── PID state ─────────────────────────────────────────────────────────────
struct PID {
  float kp = 1.5f;   // TODO: tune
  float ki = 0.5f;   // TODO: tune
  float kd = 0.05f;  // TODO: tune
  float integral  = 0.0f;
  float prevError = 0.0f;
  static constexpr float INTEGRAL_LIMIT = 200.0f;

  int compute(float target, float current, float dt) {
    float error    = target - current;
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
// RX  "C:<left_rpm>:<right_rpm>\n"   e.g.  C:45.0:-20.5
// TX  "E:<left_ticks>:<right_ticks>\n"  e.g.  E:1234:-56

void parseCommand(const String& line) {
  if (!line.startsWith("C:")) return;

  int sep = line.indexOf(':', 2);
  if (sep < 0) return;

  float l = line.substring(2, sep).toFloat();
  float r = line.substring(sep + 1).toFloat();

  noInterrupts();
  targetRpmL = l;
  targetRpmR = r;
  interrupts();
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

  // Read targets atomically
  noInterrupts();
  float tL = targetRpmL;
  float tR = targetRpmR;
  interrupts();

  // If both targets are zero, cut power immediately; no PID wind-up
  if (tL == 0.0f && tR == 0.0f) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    pidL.reset();
    pidR.reset();
  } else {
    motorL.setSpeed(pidL.compute(tL, currentRpmL, dt));
    motorR.setSpeed(pidR.compute(tR, currentRpmR, dt));
  }

  // Send encoder feedback to ROS2
  Serial.print("E:");
  Serial.print(ticksL);
  Serial.print(":");
  Serial.println(ticksR);
}
