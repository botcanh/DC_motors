#include <Arduino.h>
#include "DCMotor.h"

// Motor configuration
// direction pins: D18, D19
// enable pin: D21
// encoder pins: D22, D23
DCMotor motor(18, 19, 21, 22, 23);

uint32_t lastStatusMs = 0;

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  <number>  : set speed from -255 to 255");
  Serial.println("  z         : zero encoder count");
  Serial.println("  on        : enable motor");
  Serial.println("  off       : disable motor");
  Serial.println("  s         : print status now");
  Serial.println("  h         : print help");
}

void printStatus() {
  Serial.print("speed=");
  Serial.print(motor.getSpeed());
  Serial.print(" encoder=");
  Serial.println(motor.getEncoderCount());
}

void setup() {
  Serial.begin(115200);
  delay(300);

  motor.begin();

  Serial.println("ESP32 DC Motor + Encoder control ready");
  Serial.println("Motor: dir=GPIO18/19, enable=GPIO21, encoder=GPIO22/23");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) {
      return;
    }

    if (cmd.equalsIgnoreCase("z")) {
      motor.resetEncoder();
      Serial.println("encoder reset");
    } else if (cmd.equalsIgnoreCase("s")) {
      printStatus();
    } else if (cmd.equalsIgnoreCase("h")) {
      printHelp();
    } else if (cmd.equalsIgnoreCase("on")) {
      motor.enable(true);
      Serial.println("motor enabled");
    } else if (cmd.equalsIgnoreCase("off")) {
      motor.enable(false);
      Serial.println("motor disabled");
    } else {
      int speed = cmd.toInt();
      motor.setSpeed(speed);
      printStatus();
    }
  }

  uint32_t now = millis();
  if (now - lastStatusMs >= 500) {
    lastStatusMs = now;
    printStatus();
  }
}