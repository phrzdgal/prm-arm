// Unit tests for the two-link arm.
//
// Each test is a claim about how the arm behaves. Before running them,
// try to predict each answer by sketching the arm on paper.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "prm/two_link_arm.hpp"

using prm::degToRad;

namespace {

// 1e-9 m = one nanometre. Floating-point error in these formulas is far
// smaller than this, so a failure here is a real bug, not rounding noise.
constexpr double kTol = 1e-9;

// Unit-length links with generous limits: makes hand-checking easy.
prm::ArmParams unitArm() {
  return prm::ArmParams{
      .link1_length_m = 1.0,
      .link2_length_m = 1.0,
      .joint1 = {degToRad(-180.0), degToRad(180.0)},
      .joint2 = {degToRad(-180.0), degToRad(180.0)},
  };
}

void expectPoint(const prm::Point2D& p, double x, double y) {
  EXPECT_NEAR(p.x, x, kTol);
  EXPECT_NEAR(p.y, y, kTol);
}

}  // namespace

// --- Known configurations ----------------------------------------------------

TEST(ForwardKinematics, StraightOutAlongX) {
  expectPoint(prm::forwardKinematics(unitArm(), 0.0, 0.0), 2.0, 0.0);
}

TEST(ForwardKinematics, StraightUp) {
  expectPoint(prm::forwardKinematics(unitArm(), degToRad(90.0), 0.0), 0.0, 2.0);
}

TEST(ForwardKinematics, ElbowBentNinety) {
  // Link 1 along +x, link 2 turns up. Sketch it: an "L" shape.
  expectPoint(prm::forwardKinematics(unitArm(), 0.0, degToRad(90.0)), 1.0, 1.0);
}

TEST(ForwardKinematics, ShoulderUpElbowBent) {
  // This is the test that catches "theta2 is absolute" bugs.
  expectPoint(prm::forwardKinematics(unitArm(), degToRad(90.0), degToRad(90.0)), -1.0, 1.0);
}

TEST(ForwardKinematics, ElbowCancelsShoulder) {
  // theta1 + theta2 = 0, so link 2 is horizontal again.
  const double s = std::sqrt(2.0) / 2.0;
  expectPoint(prm::forwardKinematics(unitArm(), degToRad(45.0), degToRad(-45.0)), s + 1.0, s);
}

TEST(ForwardKinematics, FullyFoldedReturnsToBase) {
  // Equal links, elbow folded back 180 deg: the tip lands on the base.
  expectPoint(prm::forwardKinematics(unitArm(), degToRad(30.0), degToRad(180.0)), 0.0, 0.0);
}

TEST(ForwardKinematics, UnequalLinksRealisticArm) {
  // Default arm: 0.30 m + 0.25 m, theta1 = 30 deg, theta2 = 60 deg.
  // x = 0.30*cos(30) + 0.25*cos(90) = 0.259808
  // y = 0.30*sin(30) + 0.25*sin(90) = 0.400000
  const auto p = prm::forwardKinematics(prm::defaultArm(), degToRad(30.0), degToRad(60.0));
  expectPoint(p, 0.30 * std::cos(degToRad(30.0)), 0.40);
}

TEST(ElbowPosition, FollowsLinkOne) {
  expectPoint(prm::elbowPosition(prm::defaultArm(), degToRad(90.0)), 0.0, 0.30);
}

// --- Properties that must hold for ANY valid configuration --------------------

TEST(ForwardKinematics, TipStaysInsideReachableAnnulus) {
  // Whatever the angles, the tip distance from the base must lie between
  // |L1 - L2| and L1 + L2. Checking an invariant over many inputs catches
  // bugs that a handful of hand-picked cases can miss.
  const auto arm = prm::defaultArm();
  const double r_min = std::abs(arm.link1_length_m - arm.link2_length_m);
  const double r_max = arm.link1_length_m + arm.link2_length_m;
  for (int a = -90; a <= 90; a += 5) {
    for (int b = -150; b <= 150; b += 5) {
      const auto p = prm::forwardKinematics(arm, degToRad(a), degToRad(b));
      const double r = std::hypot(p.x, p.y);
      EXPECT_GE(r, r_min - kTol) << "theta1=" << a << " theta2=" << b;
      EXPECT_LE(r, r_max + kTol) << "theta1=" << a << " theta2=" << b;
    }
  }
}

TEST(ForwardKinematics, LinkTwoLengthIsPreserved) {
  // The distance from elbow to tip must always equal L2.
  const auto arm = prm::defaultArm();
  for (int a = -90; a <= 90; a += 15) {
    for (int b = -150; b <= 150; b += 15) {
      const auto e = prm::elbowPosition(arm, degToRad(a));
      const auto p = prm::forwardKinematics(arm, degToRad(a), degToRad(b));
      EXPECT_NEAR(std::hypot(p.x - e.x, p.y - e.y), arm.link2_length_m, kTol);
    }
  }
}

// --- Joint-limit validation ------------------------------------------------

TEST(Limits, InclusiveBoundaries) {
  const prm::JointLimits lim{-1.0, 1.0};
  EXPECT_TRUE(prm::withinLimits(lim, -1.0));
  EXPECT_TRUE(prm::withinLimits(lim, 0.0));
  EXPECT_TRUE(prm::withinLimits(lim, 1.0));
  EXPECT_FALSE(prm::withinLimits(lim, 1.0000001));
  EXPECT_FALSE(prm::withinLimits(lim, -1.0000001));
}

TEST(ComputeFk, AcceptsValidInput) {
  const auto arm = prm::defaultArm();
  const auto r = prm::computeFk(arm, degToRad(30.0), degToRad(60.0));
  ASSERT_EQ(r.status, prm::FkStatus::Ok);
  expectPoint(r.end_effector, 0.30 * std::cos(degToRad(30.0)), 0.40);
}

TEST(ComputeFk, RejectsJoint1OutOfLimits) {
  const auto r = prm::computeFk(prm::defaultArm(), degToRad(91.0), 0.0);
  EXPECT_EQ(r.status, prm::FkStatus::Joint1OutOfLimits);
}

TEST(ComputeFk, RejectsJoint2OutOfLimits) {
  const auto r = prm::computeFk(prm::defaultArm(), 0.0, degToRad(-151.0));
  EXPECT_EQ(r.status, prm::FkStatus::Joint2OutOfLimits);
}

TEST(ComputeFk, ReportsJoint1FirstWhenBothBad) {
  // Deterministic error ordering makes logs predictable.
  const auto r = prm::computeFk(prm::defaultArm(), degToRad(120.0), degToRad(170.0));
  EXPECT_EQ(r.status, prm::FkStatus::Joint1OutOfLimits);
}

TEST(ComputeFk, RejectsNaN) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(prm::computeFk(prm::defaultArm(), nan, 0.0).status, prm::FkStatus::NonFiniteInput);
  EXPECT_EQ(prm::computeFk(prm::defaultArm(), 0.0, nan).status, prm::FkStatus::NonFiniteInput);
}

TEST(ComputeFk, RejectsInfinity) {
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(prm::computeFk(prm::defaultArm(), inf, 0.0).status, prm::FkStatus::NonFiniteInput);
  EXPECT_EQ(prm::computeFk(prm::defaultArm(), 0.0, -inf).status, prm::FkStatus::NonFiniteInput);
}
