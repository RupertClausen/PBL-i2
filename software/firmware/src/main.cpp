// PBL-i2 quadruped firmware — entry point.
//
// Serial commands (115200 baud, newline-terminated):
//   center   move every servo to 90 deg + trim (use before fitting horns)
//   stand    stand at STAND_HEIGHT using inverse kinematics
//   relax    stop driving the servos
//   help     list commands

#include <Arduino.h>

#include "Kinematics.h"
#include "LegServos.h"
#include "robot_config.h"

enum class Mode { IDLE, STAND, WALK, CLIMB, GOALKEEP };

static LegServos servos;
static LegKinematics ik(COXA_LEN, FEMUR_LEN, TIBIA_LEN);
static Mode mode = Mode::IDLE;

static bool moveFoot(Leg leg, float x, float y, float z) {
  JointAngles a;
  if (!ik.solve(x, y, z, a)) return false;
  servos.setJoint(leg, HIP, a.hip);
  servos.setJoint(leg, UPPER, a.upper);
  servos.setJoint(leg, LOWER, a.lower);
  return true;
}

static void stand() {
  for (uint8_t l = 0; l < NUM_LEGS; l++) {
    if (!moveFoot(static_cast<Leg>(l), 0, COXA_LEN, STAND_HEIGHT)) {
      Serial.printf("Leg %u: stand pose unreachable\n", l);
    }
  }
}

static void handleCommand(const String &cmd) {
  if (cmd == "center") {
    servos.centerAll();
    mode = Mode::IDLE;
  } else if (cmd == "stand") {
    stand();
    mode = Mode::STAND;
  } else if (cmd == "relax") {
    servos.relaxAll();
    mode = Mode::IDLE;
  } else if (cmd == "help") {
    Serial.println("Commands: center, stand, relax, help");
    return;
  } else {
    Serial.printf("Unknown command '%s' (try 'help')\n", cmd.c_str());
    return;
  }
  Serial.println("OK");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  if (!servos.begin()) {
    Serial.println("PCA9685 not found — check wiring");
  }
  Serial.println("PBL-i2 ready. Type 'help'.");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length()) handleCommand(cmd);
  }

  switch (mode) {
    case Mode::WALK:      // TODO: trot / creep gait
    case Mode::CLIMB:     // TODO: stair-climbing gait (2 cm steps)
    case Mode::GOALKEEP:  // TODO: ball detection + blocking motion
    default:
      break;
  }
}
