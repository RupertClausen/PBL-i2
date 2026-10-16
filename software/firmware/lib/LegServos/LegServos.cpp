#include "LegServos.h"

#include <Wire.h>

bool LegServos::begin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!pwm_.begin()) return false;
  pwm_.setOscillatorFrequency(25000000);
  pwm_.setPWMFreq(SERVO_PWM_FREQ);
  return true;
}

void LegServos::setRaw(uint8_t channel, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  uint16_t us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f;
  pwm_.writeMicroseconds(channel, us);
}

void LegServos::setJoint(Leg leg, Joint joint, float deg) {
  float servoDeg = 90.0f + SERVO_DIR[leg][joint] * deg + SERVO_TRIM[leg][joint];
  setRaw(SERVO_CHANNEL[leg][joint], servoDeg);
}

void LegServos::centerAll() {
  for (uint8_t l = 0; l < NUM_LEGS; l++)
    for (uint8_t j = 0; j < NUM_JOINTS; j++)
      setRaw(SERVO_CHANNEL[l][j], 90.0f + SERVO_TRIM[l][j]);
}

void LegServos::relaxAll() {
  for (uint8_t l = 0; l < NUM_LEGS; l++)
    for (uint8_t j = 0; j < NUM_JOINTS; j++)
      pwm_.setPWM(SERVO_CHANNEL[l][j], 0, 0);
}
