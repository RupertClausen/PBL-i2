#include "Kinematics.h"

#include <math.h>

static constexpr float RAD2DEG = 180.0f / M_PI;

bool LegKinematics::solve(float x, float y, float z, JointAngles &out) const {
  // Frontal plane: remove the lateral coxa offset to get the leg length L
  // measured in the plane the femur and tibia move in.
  float d2 = y * y + z * z;
  if (d2 < coxa_ * coxa_) return false;
  float L = sqrtf(d2 - coxa_ * coxa_);
  float hip = atan2f(y, z) - atan2f(coxa_, L);

  // Sagittal plane: two-link IK for femur + tibia, knee bending backwards.
  float r2 = x * x + L * L;
  float r = sqrtf(r2);
  if (r > femur_ + tibia_ || r < fabsf(femur_ - tibia_)) return false;

  float cosKnee = (femur_ * femur_ + tibia_ * tibia_ - r2) / (2 * femur_ * tibia_);
  float cosAlpha = (femur_ * femur_ + r2 - tibia_ * tibia_) / (2 * femur_ * r);
  float knee = acosf(fminf(fmaxf(cosKnee, -1.0f), 1.0f));
  float alpha = acosf(fminf(fmaxf(cosAlpha, -1.0f), 1.0f));

  out.hip = hip * RAD2DEG;
  out.upper = (atan2f(x, L) + alpha) * RAD2DEG;
  out.lower = 180.0f - knee * RAD2DEG;
  return true;
}
