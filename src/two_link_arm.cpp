#include "prm/two_link_arm.hpp"

#include <cmath>

namespace prm {

Point2D elbowPosition(const ArmParams& arm, double theta1_rad) {
  // link 1 starts at the origin and points along angle theta1.
  return {(arm.link1_length_m * cos(theta1_rad)), (arm.link1_length_m * sin(theta1_rad))};
}

Point2D forwardKinematics(const ArmParams& arm, double theta1_rad, double theta2_rad) {
  // Careful: theta2 is RELATIVE to link 1. What absolute angle does
  // link 2 point along?
  Point2D p1 = elbowPosition(arm, theta1_rad);
  return {(p1.x + (arm.link2_length_m * cos(theta1_rad + theta2_rad))), 
          (p1.y + (arm.link2_length_m * sin(theta1_rad + theta2_rad)))};
}

bool withinLimits(const JointLimits& limits, double angle_rad) {
  // Inclusive check: min <= angle <= max.
  if (angle_rad >= limits.min_rad && angle_rad <= limits.max_rad) {
    return true;
  } else {
    return false;
  }
}

FkResult computeFk(const ArmParams& arm, double theta1_rad, double theta2_rad) {
  if (!std::isfinite(theta1_rad) || !std::isfinite(theta2_rad)) {
    return {FkStatus::NonFiniteInput, {}};
  }
  if (!withinLimits(arm.joint1, theta1_rad)) {
    return {FkStatus::Joint1OutOfLimits, {}};
  }
  if (!withinLimits(arm.joint2, theta2_rad)) {
    return {FkStatus::Joint2OutOfLimits, {}};
  }

  return {FkStatus::Ok, forwardKinematics(arm, theta1_rad, theta2_rad)};
}

std::string_view toString(FkStatus status) {
  switch (status) {
    case FkStatus::Ok:                return "OK";
    case FkStatus::NonFiniteInput:    return "NON_FINITE_INPUT";
    case FkStatus::Joint1OutOfLimits: return "JOINT1_OUT_OF_LIMITS";
    case FkStatus::Joint2OutOfLimits: return "JOINT2_OUT_OF_LIMITS";
  }
  return "UNKNOWN";
}

ArmParams defaultArm() {
  return ArmParams{
      .link1_length_m = 0.30,
      .link2_length_m = 0.25,
      .joint1 = {degToRad(-90.0), degToRad(90.0)},
      .joint2 = {degToRad(-150.0), degToRad(150.0)},
  };
}

}  // namespace prm
