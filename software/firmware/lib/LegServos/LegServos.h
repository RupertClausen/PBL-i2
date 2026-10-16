// Drives the 12 leg servos through a PCA9685, applying trim and direction
// from robot_config.h so the rest of the code works in "ideal" joint angles.
#pragma once

#include <Adafruit_PWMServoDriver.h>

#include "robot_config.h"

class LegServos {
 public:
  bool begin();

  // Joint angle in degrees relative to the joint's neutral position
  // (0 = neutral, which maps to the servo's 90 deg centre).
  void setJoint(Leg leg, Joint joint, float deg);

  // Raw servo angle 0..180, no trim or direction applied. Used for centring
  // servos before the horns are attached.
  void setRaw(uint8_t channel, float deg);

  void centerAll();
  void relaxAll();  // stop sending pulses so servos go limp

 private:
  Adafruit_PWMServoDriver pwm_{PCA9685_ADDR};
};
