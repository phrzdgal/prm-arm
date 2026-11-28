#pragma once
// Two-link planar arm: INVERSE kinematics.
//
// Forward kinematics (week 1):  joint angles  ->  tip position
// Inverse kinematics (week 2):  tip position  ->  joint angles
//
// Same conventions as two_link_arm.hpp: radians, metres, theta2 measured
// relative to link 1, base at the origin.

#include <string_view>

#include "prm/two_link_arm.hpp"

namespace prm {

// Most reachable points have TWO solutions: the elbow can bend either way.
// We name them by the sign of theta2:
//   ElbowDown: theta2 >= 0  (elbow bends counter-clockwise)
//   ElbowUp:   theta2 <= 0  (elbow bends clockwise)
// For a target on the +x axis, ElbowDown puts the elbow below the axis.
enum class ElbowConfig {
  Down,
  Up,
};

enum class IkStatus {
  Ok,
  NonFiniteInput,       // target x or y is NaN or infinite
  OutOfReach,           // no joint angles can put the tip there
  JointLimitsViolated,  // a solution exists, but it breaks a joint limit
};

struct IkResult {
  IkStatus status;
  double theta1_rad;  // Only meaningful when status == IkStatus::Ok
  double theta2_rad;
};

// Wrap any angle into the range [-pi, pi).
// Example: 270 deg -> -90 deg. Same physical angle, standard range.
// (Remember the torus: angles that differ by 360 deg are the same pose.)
double wrapToPi(double angle_rad);

// Can the tip physically get to this point, ignoring joint limits?
// True if the distance from the base is between |L1 - L2| and L1 + L2.
bool isReachable(const ArmParams& arm, Point2D target);

// Safe entry point: validate, solve, check joint limits.
IkResult inverseKinematics(const ArmParams& arm, Point2D target, ElbowConfig elbow);

// Human-readable status for logs and the CLI.
std::string_view toString(IkStatus status);

}  // namespace prm
