#pragma once  // tells the C++ compiler to include this header file once per compilation unit,
              // even if referenced multiple times

// Two-link planar arm: forward kinematics + joint-limit validation.
//
// Conventions (write these down in every robotics project — most bugs
// in kinematics code are convention bugs, not math bugs):
//   * Angles are in RADIANS internally. Degrees only at the user boundary.
//   * theta1 is measured from the base x-axis, counter-clockwise positive.
//   * theta2 is measured RELATIVE to link 1 (not to the base x-axis).
//   * Lengths are in metres. The base joint sits at the origin (0, 0).

#include <numbers>
#include <string_view>

namespace prm {

struct JointLimits {
  double min_rad;
  double max_rad;
};

struct ArmParams {
  double link1_length_m;
  double link2_length_m;
  JointLimits joint1;
  JointLimits joint2;
};

struct Point2D {
  double x;
  double y;
};

// Why a status code instead of throwing an exception?
// Many medical-device and real-time codebases forbid exceptions on the
// control path. An explicit status forces the caller to handle failure,
// which is the habit we want before we ever command a real motor.
enum class FkStatus {
  Ok,
  NonFiniteInput,     // NaN or infinity was passed in
  Joint1OutOfLimits,
  Joint2OutOfLimits,
};

struct FkResult {
  FkStatus status;
  Point2D end_effector;  // Only meaningful when status == FkStatus::Ok
};

// --- Unit helpers ------------------------------------------------------------
constexpr double degToRad(double deg) { return deg * std::numbers::pi / 180.0; }
constexpr double radToDeg(double rad) { return rad * 180.0 / std::numbers::pi; }

// --- Core API ----------------------------------------------------------------

// Pure math: where is the end effector for these joint angles?
// Does NOT check limits. Use this when you already know the input is valid.
Point2D forwardKinematics(const ArmParams& arm, double theta1_rad, double theta2_rad);

// Where is the elbow joint? (Handy for drawing the arm and for later
// collision checks.)
Point2D elbowPosition(const ArmParams& arm, double theta1_rad);

// Is this angle inside the given limits (inclusive)?
bool withinLimits(const JointLimits& limits, double angle_rad);

// Safe entry point: validates inputs, then computes forward kinematics.
FkResult computeFk(const ArmParams& arm, double theta1_rad, double theta2_rad);

// Human-readable status for logs and the CLI.
std::string_view toString(FkStatus status);

// A reasonable default arm for experiments: 0.30 m + 0.25 m links,
// shoulder ±90°, elbow ±150° (the elbow never folds flat onto link 1).
ArmParams defaultArm();

}  // namespace prm
