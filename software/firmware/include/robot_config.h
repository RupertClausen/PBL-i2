// Robot-wide configuration: pins, dimensions and calibration.
// Keep this in sync with hardware/electronics/README.md.
#pragma once

#include <stdint.h>

// ---------- Communication ----------
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// ---------- PCA9685 servo driver ----------
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr float SERVO_PWM_FREQ = 50.0f;  // Hz, standard analog servos
constexpr uint16_t SERVO_MIN_US = 500;   // pulse width at 0 deg
constexpr uint16_t SERVO_MAX_US = 2500;  // pulse width at 180 deg

// ---------- Leg geometry (mm) — update from CAD ----------
constexpr float COXA_LEN = 30.0f;   // hip axis -> upper joint
constexpr float FEMUR_LEN = 60.0f;  // upper joint -> knee
constexpr float TIBIA_LEN = 70.0f;  // knee -> foot

// ---------- Legs and joints ----------
enum Leg : uint8_t { FL = 0, FR, RL, RR, NUM_LEGS };
enum Joint : uint8_t { HIP = 0, UPPER, LOWER, NUM_JOINTS };

// PCA9685 channel for each [leg][joint]
constexpr uint8_t SERVO_CHANNEL[NUM_LEGS][NUM_JOINTS] = {
    {0, 1, 2},    // FL
    {3, 4, 5},    // FR
    {6, 7, 8},    // RL
    {9, 10, 11},  // RR
};

// Trim offsets in degrees, found during calibration
constexpr float SERVO_TRIM[NUM_LEGS][NUM_JOINTS] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
};

// +1 or -1: flips the direction of mirrored servos
constexpr int8_t SERVO_DIR[NUM_LEGS][NUM_JOINTS] = {
    {+1, +1, +1},
    {-1, -1, -1},
    {+1, +1, +1},
    {-1, -1, -1},
};

// ---------- Default stance ----------
constexpr float STAND_HEIGHT = 90.0f;  // mm, hip to foot
