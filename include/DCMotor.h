#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

class DCMotor {
public:
  /**
   * Initialize DC motor with encoder and PWM control.
   * @param dirPin1 GPIO pin for direction (forward)
   * @param dirPin2 GPIO pin for direction (reverse)
   * @param enablePin GPIO pin for PWM speed control
   * @param encoderPinA GPIO pin for encoder channel A
   * @param encoderPinB GPIO pin for encoder channel B
   */
  DCMotor(int dirPin1, int dirPin2, int enablePin, int encoderPinA, int encoderPinB);

  /**
   * Initialize motor (call once in setup).
   */
  void begin();

  /**
   * Set motor speed and direction.
   * @param speed -255 (full reverse) to +255 (full forward), 0 = stop
   */
  void setSpeed(int speed);

  /**
   * Get current commanded speed.
   */
  int getSpeed() const { return commandedSpeed; }

  /**
   * Get encoder count.
   */
  long getEncoderCount() const;

  /**
   * Reset encoder count to zero.
   */
  void resetEncoder();

  /**
   * Enable or disable motor (EN pin control).
   */
  void enable(bool en);

private:
  // Pin configuration
  int dirPin1;
  int dirPin2;
  int enablePin;
  int encoderPinA;
  int encoderPinB;

  // PWM settings
  static constexpr int PWM_FREQ_HZ = 20000;
  static constexpr int PWM_RESOLUTION_BITS = 8;
  static constexpr int PWM_MAX = 255;
  static constexpr int PWM_CHANNEL = 0;

  // Encoder state
  volatile long encoderCount;
  volatile uint8_t prevEncoderState;
  int commandedSpeed;
  bool isEnabled;

  // Static pointer for ISR
  static DCMotor *instance;

  // Encoder ISR (static)
  static void IRAM_ATTR onEncoderChangeISR();

  // Encoder ISR (instance method)
  void onEncoderChange();

  // Helper
  uint8_t readEncoderState() const;
};

#endif
