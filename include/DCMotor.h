#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

class DCMotor {
public:
  DCMotor(int dirPin1, int dirPin2, int enablePin, int encoderPinA, int encoderPinB, int pwmChannel);

  void begin();

  // Set PWM speed: -255 (full reverse) to +255 (full forward), 0 = stop
  void setSpeed(int speed);

  int  getSpeed()        const { return commandedSpeed; }
  long getEncoderCount() const;
  void resetEncoder();
  void enable(bool en);

private:
  int dirPin1, dirPin2, enablePin, encoderPinA, encoderPinB;
  int pwmChannel;

  static constexpr int PWM_FREQ_HZ        = 20000;
  static constexpr int PWM_RESOLUTION_BITS = 8;
  static constexpr int PWM_MAX             = 255;

  volatile long encoderCount;
  volatile uint8_t prevEncoderState;
  int  commandedSpeed;
  bool isEnabled;
  int  motorIndex;

  // Supports up to 2 motors
  static DCMotor* instances[2];
  static int      instanceCount;

  static void IRAM_ATTR isr0();
  static void IRAM_ATTR isr1();
  void onEncoderChange();

  uint8_t readEncoderState() const;
};

#endif
