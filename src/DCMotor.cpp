#include "DCMotor.h"

// Initialize static pointer
DCMotor *DCMotor::instance = nullptr;

DCMotor::DCMotor(int dirPin1, int dirPin2, int enablePin, int encoderPinA, int encoderPinB)
    : dirPin1(dirPin1), dirPin2(dirPin2), enablePin(enablePin),
      encoderPinA(encoderPinA), encoderPinB(encoderPinB),
      encoderCount(0), prevEncoderState(0), commandedSpeed(0), isEnabled(true) {
  instance = this;
}

void DCMotor::begin() {
  // Configure direction pins as outputs
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Configure encoder pins as inputs with pull-ups
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Setup PWM on enable pin
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(enablePin, PWM_CHANNEL);

  // Initialize encoder state
  prevEncoderState = readEncoderState();

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA), onEncoderChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), onEncoderChangeISR, CHANGE);

  // Start with motor stopped
  setSpeed(0);
}

void DCMotor::setSpeed(int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  commandedSpeed = speed;

  if (!isEnabled) {
    ledcWrite(PWM_CHANNEL, 0);
    return;
  }

  if (speed > 0) {
    // Forward
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    ledcWrite(PWM_CHANNEL, speed);
  } else if (speed < 0) {
    // Reverse
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    ledcWrite(PWM_CHANNEL, -speed);
  } else {
    // Stop
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}

long DCMotor::getEncoderCount() const {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

void DCMotor::resetEncoder() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

void DCMotor::enable(bool en) {
  isEnabled = en;
  if (!isEnabled) {
    ledcWrite(PWM_CHANNEL, 0);
  } else {
    setSpeed(commandedSpeed);
  }
}

uint8_t DCMotor::readEncoderState() const {
  uint8_t a = static_cast<uint8_t>(digitalRead(encoderPinA));
  uint8_t b = static_cast<uint8_t>(digitalRead(encoderPinB));
  return static_cast<uint8_t>((a << 1) | b);
}

void IRAM_ATTR DCMotor::onEncoderChangeISR() {
  if (instance != nullptr) {
    instance->onEncoderChange();
  }
}

void DCMotor::onEncoderChange() {
  // Transition table for quadrature decoding
  static const int8_t table[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0};

  uint8_t newState = readEncoderState();
  uint8_t idx = static_cast<uint8_t>((prevEncoderState << 2) | newState);
  encoderCount += table[idx];
  prevEncoderState = newState;
}
