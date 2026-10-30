#include "StairClimb.h"

#include <Arduino.h>

const char *stairStateName(StairState s) {
  switch (s) {
    case StairState::START:           return "S0 start";
    case StairState::INIT:            return "S1 init";
    case StairState::INITIAL_POSE:    return "S2 initial pose";
    case StairState::UP_SHIFT_COM:    return "U1 shift CoM";
    case StairState::UP_FRONT_LEGS:   return "U2 front legs up";
    case StairState::UP_MOVE_BODY:    return "U3 body forward/up";
    case StairState::UP_REAR_LEGS:    return "U4 rear legs up";
    case StairState::CHECK_UP:        return "A climbed enough?";
    case StairState::STABILIZE_TOP:   return "S3 stabilize on top";
    case StairState::DESCENT_START:   return "S4 move to descent start";
    case StairState::DOWN_SHIFT_COM:  return "D1 shift CoM";
    case StairState::DOWN_FRONT_LEGS: return "D2 front legs down";
    case StairState::DOWN_MOVE_BODY:  return "D3 body forward/down";
    case StairState::DOWN_REAR_LEGS:  return "D4 rear legs down";
    case StairState::CHECK_DOWN:      return "B descended enough?";
    case StairState::FINAL_POSE:      return "S5 final pose";
    case StairState::GOAL:            return "S6 goal";
  }
  return "?";
}

void StairClimber::start() {
  running_ = true;
  enter(StairState::START);
}

void StairClimber::abort() {
  if (running_) Serial.printf("[climb] aborted in %s\n", stairStateName(state_));
  running_ = false;
}

void StairClimber::enter(StairState next) {
  state_ = next;
  Serial.printf("[climb] %s\n", stairStateName(next));
}

bool StairClimber::update() {
  if (!running_) return state_ == StairState::GOAL;

  switch (state_) {
    case StairState::START:
      enter(StairState::INIT);
      break;
    case StairState::INIT:
      if (initialize()) enter(StairState::INITIAL_POSE);
      break;
    case StairState::INITIAL_POSE:
      if (setInitialPose()) enter(StairState::UP_SHIFT_COM);
      break;

    // ---- Up (crawl gait) ----
    case StairState::UP_SHIFT_COM:
      if (shiftCenterOfMass(supportSide_)) enter(StairState::UP_FRONT_LEGS);
      break;
    case StairState::UP_FRONT_LEGS:
      if (liftFrontLegsToUpperStep()) enter(StairState::UP_MOVE_BODY);
      break;
    case StairState::UP_MOVE_BODY:
      if (moveBodyForwardUp()) enter(StairState::UP_REAR_LEGS);
      break;
    case StairState::UP_REAR_LEGS:
      if (liftRearLegsToUpperStep()) {
        stepsUp_++;
        enter(StairState::CHECK_UP);
      }
      break;
    case StairState::CHECK_UP:
      Serial.printf("[climb] climbed %d/%d\n", stepsUp_, STAIR_STEP_COUNT);
      if (reachedStepCount(stepsUp_, STAIR_STEP_COUNT)) {
        enter(StairState::STABILIZE_TOP);
      } else {
        supportSide_ = nextSupportSide();
        enter(StairState::UP_SHIFT_COM);
      }
      break;

    case StairState::STABILIZE_TOP:
      if (stabilizeOnTop()) enter(StairState::DESCENT_START);
      break;
    case StairState::DESCENT_START:
      if (moveToDescentStart()) enter(StairState::DOWN_SHIFT_COM);
      break;

    // ---- Down (crawl gait) ----
    case StairState::DOWN_SHIFT_COM:
      if (shiftCenterOfMass(supportSide_)) enter(StairState::DOWN_FRONT_LEGS);
      break;
    case StairState::DOWN_FRONT_LEGS:
      if (lowerFrontLegsToLowerStep()) enter(StairState::DOWN_MOVE_BODY);
      break;
    case StairState::DOWN_MOVE_BODY:
      if (moveBodyForwardDown()) enter(StairState::DOWN_REAR_LEGS);
      break;
    case StairState::DOWN_REAR_LEGS:
      if (lowerRearLegsToLowerStep()) {
        stepsDown_++;
        enter(StairState::CHECK_DOWN);
      }
      break;
    case StairState::CHECK_DOWN:
      Serial.printf("[climb] descended %d/%d\n", stepsDown_, STAIR_STEP_COUNT);
      if (reachedStepCount(stepsDown_, STAIR_STEP_COUNT)) {
        enter(StairState::FINAL_POSE);
      } else {
        supportSide_ = nextSupportSide();
        enter(StairState::DOWN_SHIFT_COM);
      }
      break;

    case StairState::FINAL_POSE:
      if (returnToStablePose()) enter(StairState::GOAL);
      break;
    case StairState::GOAL:
      running_ = false;
      break;
  }
  return state_ == StairState::GOAL;
}

// ---------------------------------------------------------------------------
// Steps — all stubs. Return true when the motion is finished.
// ---------------------------------------------------------------------------

bool StairClimber::initialize() {
  stepsUp_ = 0;
  stepsDown_ = 0;
  supportSide_ = Side::LEFT;
  // TODO: Arduino and PCA9685 are already set up in setup(); decide whether
  //       to re-check the servo driver / IMU here before starting.
  return true;
}

bool StairClimber::setInitialPose() {
  // TODO: put all four feet in the initial standing pose for the stairs
  //       (height, foot spread, distance to the first step undecided).
  return true;
}

bool StairClimber::shiftCenterOfMass(Side supportSide) {
  (void)supportSide;
  // TODO: move the body over the support legs on `supportSide` so the legs
  //       about to be lifted carry no weight. Shift amount undecided.
  return true;
}

bool StairClimber::liftFrontLegsToUpperStep() {
  // TODO: lift FL and FR one at a time onto the upper step (foot trajectory,
  //       step height and ground-contact detection undecided). Possibly shift
  //       the CoM again between the two legs.
  return true;
}

bool StairClimber::moveBodyForwardUp() {
  // TODO: move the body forward and up onto the step with all feet planted.
  return true;
}

bool StairClimber::liftRearLegsToUpperStep() {
  // TODO: lift RL and RR one at a time onto the upper step.
  return true;
}

bool StairClimber::stabilizeOnTop() {
  // TODO: settle the posture on the top step (IMU levelling?).
  return true;
}

bool StairClimber::moveToDescentStart() {
  // TODO: walk / turn to the edge where the descent begins.
  return true;
}

bool StairClimber::lowerFrontLegsToLowerStep() {
  // TODO: lower FL and FR one at a time onto the lower step.
  return true;
}

bool StairClimber::moveBodyForwardDown() {
  // TODO: move the body forward and down with all feet planted.
  return true;
}

bool StairClimber::lowerRearLegsToLowerStep() {
  // TODO: lower RL and RR one at a time onto the lower step.
  return true;
}

bool StairClimber::returnToStablePose() {
  // TODO: return all legs to a stable standing pose on the floor.
  return true;
}

bool StairClimber::reachedStepCount(int count, int target) const {
  // TODO: only counts completed U1-U4 / D1-D4 cycles for now; may later be
  //       confirmed with a sensor.
  return count >= target;
}

Side StairClimber::nextSupportSide() {
  // TODO: support-side order for the crawl gait is undecided; alternate for now.
  return supportSide_ == Side::LEFT ? Side::RIGHT : Side::LEFT;
}

bool StairClimber::moveFoot(Leg leg, float x, float y, float z) {
  JointAngles a;
  if (!ik_.solve(x, y, z, a)) return false;
  servos_.setJoint(leg, HIP, a.hip);
  servos_.setJoint(leg, UPPER, a.upper);
  servos_.setJoint(leg, LOWER, a.lower);
  return true;
}
