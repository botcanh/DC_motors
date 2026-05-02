#include "DCMotor.h"

DCMotor* DCMotor::instances[2]  = {nullptr, nullptr};
int      DCMotor::instanceCount = 0;

DCMotor::DCMotor(int dirPin1, int dirPin2, int enablePin,
                 int encoderPinA, int encoderPinB, int pwmChannel)
    : dirPin1(dirPin1), dirPin2(dirPin2), enablePin(enablePin),
      encoderPinA(encoderPinA), encoderPinB(encoderPinB),
      pwmChannel(pwmChannel),
      encoderCount(0), prevEncoderState(0), commandedSpeed(0), isEnabled(true)
{
  motorIndex = instanceCount++;
  if (motorIndex < 2) {
    instances[motorIndex] = this;
  }
}

void DCMotor::begin() {
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  ledcSetup(pwmChannel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(enablePin, pwmChannel);

  prevEncoderState = readEncoderState();

  if (motorIndex == 0) {
    attachInterrupt(digitalPinToInterrupt(encoderPinA), isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), isr0, CHANGE);
  } else {
    attachInterrupt(digitalPinToInterrupt(encoderPinA), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), isr1, CHANGE);
  }

  setSpeed(0);
}

void DCMotor::setSpeed(int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  commandedSpeed = speed;

  if (!isEnabled) {
    ledcWrite(pwmChannel, 0);
    return;
  }

  if (speed > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    ledcWrite(pwmChannel, 0);
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
    ledcWrite(pwmChannel, 0);
  } else {
    setSpeed(commandedSpeed);
  }
}

uint8_t DCMotor::readEncoderState() const {
  uint8_t a = static_cast<uint8_t>(digitalRead(encoderPinA));
  uint8_t b = static_cast<uint8_t>(digitalRead(encoderPinB));
  return static_cast<uint8_t>((a << 1) | b);
}

void IRAM_ATTR DCMotor::isr0() {
  if (instances[0]) instances[0]->onEncoderChange();
}

void IRAM_ATTR DCMotor::isr1() {
  if (instances[1]) instances[1]->onEncoderChange();
}

void DCMotor::onEncoderChange() {
  static const int8_t table[16] = {
       0, -1,  1,  0,
       1,  0,  0, -1,
      -1,  0,  0,  1,
       0,  1, -1,  0
  };
  uint8_t newState = readEncoderState();
  uint8_t idx      = static_cast<uint8_t>((prevEncoderState << 2) | newState);
  encoderCount    += table[idx];
  prevEncoderState = newState;
}
