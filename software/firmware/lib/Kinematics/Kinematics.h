// Inverse kinematics for a 3-DOF leg (hip abduction, upper, lower).
#pragma once

struct JointAngles {
  float hip;    // deg, abduction; 0 = leg straight down
  float upper;  // deg, thigh pitch; 0 = femur straight down
  float lower;  // deg, knee; 0 = tibia in line with femur
};

class LegKinematics {
 public:
  LegKinematics(float coxa, float femur, float tibia)
      : coxa_(coxa), femur_(femur), tibia_(tibia) {}

  // Foot position relative to the hip axis, in mm:
  //   x forward, y outward (away from body), z down.
  // Returns false (and leaves `out` untouched) if the point is unreachable.
  bool solve(float x, float y, float z, JointAngles &out) const;

 private:
  float coxa_, femur_, tibia_;
};
