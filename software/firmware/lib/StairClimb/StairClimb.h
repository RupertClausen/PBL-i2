// Stair-climbing state machine: climb STAIR_STEP_COUNT steps with a crawl
// gait, then walk back down them.
//
// Skeleton only — every motion is a stub with a TODO. Each stub returns true
// when its motion is finished, so a stub can later become a multi-cycle
// (non-blocking) motion by returning false until it is done.
//
// Flow (from the stair-climbing flowchart):
//   S0 start -> S1 init -> S2 initial pose
//   up:   U1 shift CoM -> U2 front legs up -> U3 body forward/up
//         -> U4 rear legs up -> [A] climbed N steps? no: U1, yes: S3
//   S3 stabilise on top -> S4 move to descent start
//   down: D1 shift CoM -> D2 front legs down -> D3 body forward/down
//         -> D4 rear legs down -> [B] descended N steps? no: D1, yes: S5
//   S5 final pose -> S6 goal
#pragma once

#include "Kinematics.h"
#include "LegServos.h"
#include "robot_config.h"

enum class StairState : uint8_t {
  START,            // S0
  INIT,             // S1
  INITIAL_POSE,     // S2
  UP_SHIFT_COM,     // U1
  UP_FRONT_LEGS,    // U2
  UP_MOVE_BODY,     // U3
  UP_REAR_LEGS,     // U4
  CHECK_UP,         // A: climbed STAIR_STEP_COUNT steps?
  STABILIZE_TOP,    // S3
  DESCENT_START,    // S4
  DOWN_SHIFT_COM,   // D1
  DOWN_FRONT_LEGS,  // D2
  DOWN_MOVE_BODY,   // D3
  DOWN_REAR_LEGS,   // D4
  CHECK_DOWN,       // B: descended STAIR_STEP_COUNT steps?
  FINAL_POSE,       // S5
  GOAL,             // S6
};

enum class Side : uint8_t { LEFT, RIGHT };

const char *stairStateName(StairState s);

class StairClimber {
 public:
  StairClimber(LegServos &servos, const LegKinematics &ik)
      : servos_(servos), ik_(ik) {}

  void start();  // (re)start the sequence from S0
  void abort();  // stop where we are; does not move the servos

  // Advance the state machine by at most one transition. Call every loop().
  // Returns true once the sequence has reached S6 (goal).
  bool update();

  StairState state() const { return state_; }
  bool running() const { return running_; }

 private:
  void enter(StairState next);

  // ---- Steps (stubs) ----
  bool initialize();                            // S1
  bool setInitialPose();                        // S2
  bool shiftCenterOfMass(Side supportSide);     // U1 / D1
  bool liftFrontLegsToUpperStep();              // U2
  bool moveBodyForwardUp();                     // U3
  bool liftRearLegsToUpperStep();               // U4
  bool stabilizeOnTop();                        // S3
  bool moveToDescentStart();                    // S4
  bool lowerFrontLegsToLowerStep();             // D2
  bool moveBodyForwardDown();                   // D3
  bool lowerRearLegsToLowerStep();              // D4
  bool returnToStablePose();                    // S5
  bool reachedStepCount(int count, int target) const;  // A / B

  Side nextSupportSide();

  // Move one foot via IK (same frame as LegKinematics::solve). For the stubs
  // to use once foot trajectories are decided.
  bool moveFoot(Leg leg, float x, float y, float z);

  LegServos &servos_;
  const LegKinematics &ik_;
  StairState state_ = StairState::START;
  bool running_ = false;
  int stepsUp_ = 0;    // steps climbed so far (judgement A)
  int stepsDown_ = 0;  // steps descended so far (judgement B)
  Side supportSide_ = Side::LEFT;
};
