#include "prm/two_link_ik.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>

namespace prm {

double wrapToPi(double angle_rad) {
  const double pi = std::numbers::pi;
  while (angle_rad >= pi) angle_rad -= 2.0 * pi;
  while (angle_rad < -pi) angle_rad += 2.0 * pi;
  return angle_rad;
}

bool isReachable(const ArmParams& arm, Point2D target) {
  const double r = std::hypot(target.x, target.y);
  double longestReach = arm.link1_length_m + arm.link2_length_m;
  double closestReach = std::abs(arm.link1_length_m - arm.link2_length_m);

  constexpr double kEps = 1e-12;
  return r <= longestReach + kEps && r >= closestReach - kEps;
}

IkResult inverseKinematics(const ArmParams& arm, Point2D target, ElbowConfig elbow) {
  if (!std::isfinite(target.x) || !std::isfinite(target.y)) {
    return {IkStatus::NonFiniteInput, 0.0, 0.0};
  }
  if (!isReachable(arm, target)) {
    return {IkStatus::OutOfReach, 0.0, 0.0};
  }

  double D_raw = (target.x * target.x + target.y * target.y - arm.link1_length_m * arm.link1_length_m - arm.link2_length_m * arm.link2_length_m) / (2 * arm.link1_length_m * arm.link2_length_m);
  const double D = std::clamp(D_raw, -1.0, 1.0);
  const double theta2_rad = (elbow == ElbowConfig::Down) ? std::acos(D) : -std::acos(D);

  const double theta1_raw = atan2(target.y, target.x) - atan2(arm.link2_length_m * sin(theta2_rad), arm.link1_length_m + arm.link2_length_m * cos(theta2_rad));
  const double theta1_rad = wrapToPi(theta1_raw);

  if (!withinLimits(arm.joint1, theta1_rad) || !withinLimits(arm.joint2, theta2_rad)) {
    return {IkStatus::JointLimitsViolated, 0.0, 0.0};
  }

  return {IkStatus::Ok, theta1_rad, theta2_rad};
}

std::string_view toString(IkStatus status) {
  switch (status) {
    case IkStatus::Ok:                  return "OK";
    case IkStatus::NonFiniteInput:      return "NON_FINITE_INPUT";
    case IkStatus::OutOfReach:          return "OUT_OF_REACH";
    case IkStatus::JointLimitsViolated: return "JOINT_LIMITS_VIOLATED";
  }
  return "UNKNOWN";
}

}  
