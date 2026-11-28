// Unit tests for two-link inverse kinematics.
//
// The strongest test of IK is a round trip: take a target, solve IK,
// run your week-1 forward kinematics on the answer, and check that you
// land back on the target.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "prm/two_link_arm.hpp"
#include "prm/two_link_ik.hpp"

using prm::degToRad;

namespace {

constexpr double kTol = 1e-9;
constexpr double kPi = std::numbers::pi;

prm::ArmParams unitArm() {
  return prm::ArmParams{
      .link1_length_m = 1.0,
      .link2_length_m = 1.0,
      .joint1 = {degToRad(-180.0), degToRad(180.0)},
      .joint2 = {degToRad(-180.0), degToRad(180.0)},
  };
}

void expectAngles(const prm::IkResult& r, double t1_deg, double t2_deg) {
  ASSERT_EQ(r.status, prm::IkStatus::Ok);
  EXPECT_NEAR(r.theta1_rad, degToRad(t1_deg), kTol);
  EXPECT_NEAR(r.theta2_rad, degToRad(t2_deg), kTol);
}

}  // namespace

// --- wrapToPi ------------------------------------------------------------------

TEST(WrapToPi, LeavesInRangeAnglesAlone) {
  EXPECT_NEAR(prm::wrapToPi(0.0), 0.0, kTol);
  EXPECT_NEAR(prm::wrapToPi(degToRad(90.0)), degToRad(90.0), kTol);
  EXPECT_NEAR(prm::wrapToPi(degToRad(-170.0)), degToRad(-170.0), kTol);
}

TEST(WrapToPi, WrapsLargeAngles) {
  EXPECT_NEAR(prm::wrapToPi(degToRad(270.0)), degToRad(-90.0), kTol);
  EXPECT_NEAR(prm::wrapToPi(degToRad(-270.0)), degToRad(90.0), kTol);
  EXPECT_NEAR(prm::wrapToPi(degToRad(360.0)), 0.0, kTol);
  EXPECT_NEAR(prm::wrapToPi(degToRad(370.0)), degToRad(10.0), kTol);
  EXPECT_NEAR(prm::wrapToPi(degToRad(-730.0)), degToRad(-10.0), kTol);
}

TEST(WrapToPi, AlwaysLandsInRange) {
  for (int deg = -1080; deg <= 1080; deg += 7) {
    const double in = degToRad(deg);
    const double w = prm::wrapToPi(in);
    EXPECT_GE(w, -kPi) << "input " << deg << " deg";
    EXPECT_TRUE(w < kPi) << "input " << deg << " deg";
    // Same physical angle: cos and sin must not change.
    EXPECT_NEAR(std::cos(w), std::cos(in), kTol) << "input " << deg << " deg";
    EXPECT_NEAR(std::sin(w), std::sin(in), kTol) << "input " << deg << " deg";
  }
}

// --- isReachable -----------------------------------------------------------------

TEST(IsReachable, InsideTheRing) {
  EXPECT_TRUE(prm::isReachable(prm::defaultArm(), {0.40, 0.0}));
  EXPECT_TRUE(prm::isReachable(prm::defaultArm(), {0.0, 0.30}));
}

TEST(IsReachable, TooFar) {
  EXPECT_FALSE(prm::isReachable(prm::defaultArm(), {0.56, 0.0}));
  EXPECT_FALSE(prm::isReachable(prm::defaultArm(), {0.40, 0.40}));
}

TEST(IsReachable, TooCloseToTheBase) {
  // Links of 0.30 m and 0.25 m can't fold closer than 0.05 m.
  EXPECT_FALSE(prm::isReachable(prm::defaultArm(), {0.01, 0.0}));
}

TEST(IsReachable, ExactlyOnTheEdgesCounts) {
  EXPECT_TRUE(prm::isReachable(prm::defaultArm(), {0.55, 0.0}));
  EXPECT_TRUE(prm::isReachable(prm::defaultArm(), {0.0, 0.05}));
}

// --- Known solutions (sketch these first!) -------------------------------------

TEST(InverseKinematics, StraightOut) {
  expectAngles(prm::inverseKinematics(unitArm(), {2.0, 0.0}, prm::ElbowConfig::Down), 0.0, 0.0);
}

TEST(InverseKinematics, StraightUp) {
  expectAngles(prm::inverseKinematics(unitArm(), {0.0, 2.0}, prm::ElbowConfig::Down), 90.0, 0.0);
}

TEST(InverseKinematics, ElbowDownSolution) {
  // Same "L" shape as the week-1 test ElbowBentNinety, solved backwards.
  expectAngles(prm::inverseKinematics(unitArm(), {1.0, 1.0}, prm::ElbowConfig::Down), 0.0, 90.0);
}

TEST(InverseKinematics, ElbowUpSolution) {
  // Same target, elbow bent the other way: shoulder up, elbow back.
  expectAngles(prm::inverseKinematics(unitArm(), {1.0, 1.0}, prm::ElbowConfig::Up), 90.0, -90.0);
}

// --- Round trips: the real proof ----------------------------------------------

TEST(InverseKinematics, RoundTripLandsOnTarget) {
  // For a grid of targets, solve IK with both elbows; whenever a solution
  // is returned, forward kinematics must put the tip back on the target.
  const auto arm = prm::defaultArm();
  int solved = 0;
  for (int i = -55; i <= 55; i += 2) {
    for (int j = -55; j <= 55; j += 2) {
      const prm::Point2D target{i / 100.0, j / 100.0};
      for (auto elbow : {prm::ElbowConfig::Down, prm::ElbowConfig::Up}) {
        const auto r = prm::inverseKinematics(arm, target, elbow);
        if (r.status != prm::IkStatus::Ok) continue;
        ++solved;
        const auto tip = prm::forwardKinematics(arm, r.theta1_rad, r.theta2_rad);
        EXPECT_NEAR(tip.x, target.x, kTol) << "target (" << target.x << ", " << target.y << ")";
        EXPECT_NEAR(tip.y, target.y, kTol) << "target (" << target.x << ", " << target.y << ")";
      }
    }
  }
  EXPECT_GE(solved, 500);  // Guards against an IK that refuses everything.
}

TEST(InverseKinematics, RecoversTheAnglesThatMadeTheTarget) {
  // Go the other way: start from angles, compute the tip with FK,
  // and check IK gives the same angles back.
  const auto arm = prm::defaultArm();
  for (int a = -80; a <= 80; a += 10) {
    for (int b = 10; b <= 140; b += 10) {  // stop short of the 150 deg limit:
                                           // rounding at the exact limit can tip it over
      for (int sign : {+1, -1}) {
        const double t1 = degToRad(a);
        const double t2 = degToRad(sign * b);
        const auto tip = prm::forwardKinematics(arm, t1, t2);
        const auto elbow = sign > 0 ? prm::ElbowConfig::Down : prm::ElbowConfig::Up;
        const auto r = prm::inverseKinematics(arm, tip, elbow);
        ASSERT_EQ(r.status, prm::IkStatus::Ok);
        EXPECT_NEAR(r.theta1_rad, t1, kTol) << "theta1=" << a << " theta2=" << sign * b;
        EXPECT_NEAR(r.theta2_rad, t2, kTol) << "theta1=" << a << " theta2=" << sign * b;
      }
    }
  }
}

TEST(InverseKinematics, ElbowSignMatchesConfig) {
  const auto arm = prm::defaultArm();
  const auto down = prm::inverseKinematics(arm, {0.35, 0.10}, prm::ElbowConfig::Down);
  const auto up = prm::inverseKinematics(arm, {0.35, 0.10}, prm::ElbowConfig::Up);
  ASSERT_EQ(down.status, prm::IkStatus::Ok);
  ASSERT_EQ(up.status, prm::IkStatus::Ok);
  EXPECT_TRUE(down.theta2_rad > 0.0);
  EXPECT_TRUE(up.theta2_rad < 0.0);
}

TEST(InverseKinematics, FullyStretchedArmStillSolves) {
  // With the arm straight, the target sits exactly on the outer edge.
  // Rounding can make D come out as 1.0000000000000002, and acos of that
  // is NaN. This test fails unless D is clamped into [-1, 1].
  const auto arm = prm::defaultArm();
  for (int a = -90; a <= 90; ++a) {
    const auto tip = prm::forwardKinematics(arm, degToRad(a), 0.0);
    const auto r = prm::inverseKinematics(arm, tip, prm::ElbowConfig::Down);
    ASSERT_EQ(r.status, prm::IkStatus::Ok);
    EXPECT_NEAR(r.theta1_rad, degToRad(a), 1e-6) << "theta1=" << a;
    EXPECT_NEAR(r.theta2_rad, 0.0, 1e-6) << "theta1=" << a;
  }
}

TEST(InverseKinematics, WrapsShoulderAngleBeforeCheckingLimits) {
  // Target at -170 deg from the base. Unwrapped, theta1 comes out as
  // -215 deg, which looks out of limits. It's the same pose as +145 deg,
  // which is fine. This test fails unless theta1 is wrapped.
  const double d = std::sqrt(2.0);
  const prm::Point2D target{d * std::cos(degToRad(-170.0)), d * std::sin(degToRad(-170.0))};
  expectAngles(prm::inverseKinematics(unitArm(), target, prm::ElbowConfig::Down), 145.0, 90.0);
}

// --- Rejections ----------------------------------------------------------------

TEST(InverseKinematics, RejectsUnreachableTargets) {
  const auto arm = prm::defaultArm();
  EXPECT_EQ(prm::inverseKinematics(arm, {0.60, 0.0}, prm::ElbowConfig::Down).status,
            prm::IkStatus::OutOfReach);
  EXPECT_EQ(prm::inverseKinematics(arm, {0.01, 0.0}, prm::ElbowConfig::Up).status,
            prm::IkStatus::OutOfReach);
}

TEST(InverseKinematics, RejectsReachableButOutsideJointLimits) {
  // Behind the base: the tip can geometrically get there, but only by
  // turning the shoulder past its +/-90 deg limit.
  const auto arm = prm::defaultArm();
  EXPECT_EQ(prm::inverseKinematics(arm, {-0.40, 0.0}, prm::ElbowConfig::Down).status,
            prm::IkStatus::JointLimitsViolated);
  EXPECT_EQ(prm::inverseKinematics(arm, {-0.40, 0.0}, prm::ElbowConfig::Up).status,
            prm::IkStatus::JointLimitsViolated);
}

TEST(InverseKinematics, RejectsNonFiniteTargets) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto arm = prm::defaultArm();
  EXPECT_EQ(prm::inverseKinematics(arm, {nan, 0.1}, prm::ElbowConfig::Down).status,
            prm::IkStatus::NonFiniteInput);
  EXPECT_EQ(prm::inverseKinematics(arm, {0.1, inf}, prm::ElbowConfig::Up).status,
            prm::IkStatus::NonFiniteInput);
}
