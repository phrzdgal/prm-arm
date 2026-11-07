// Command-line demo for the two-link arm.
//
// Usage:
//   ./fk_cli <theta1_deg> <theta2_deg>     one-shot
//   ./fk_cli                               interactive (type 'q' to quit)

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "prm/two_link_arm.hpp"

namespace {

// Strict number parsing: "30abc" is rejected, not silently read as 30.
bool parseDouble(const std::string& text, double& out) {
  if (text.empty()) return false;
  char* end = nullptr;
  out = std::strtod(text.c_str(), &end);
  return end != text.c_str() && *end == '\0';
}

int report(const prm::ArmParams& arm, double t1_deg, double t2_deg) {
  const double t1 = prm::degToRad(t1_deg);
  const double t2 = prm::degToRad(t2_deg);
  const prm::FkResult r = prm::computeFk(arm, t1, t2);

  std::printf("theta1 = %8.3f deg   theta2 = %8.3f deg\n", t1_deg, t2_deg);
  if (r.status != prm::FkStatus::Ok) {
    std::printf("  REJECTED: %s\n", std::string(prm::toString(r.status)).c_str());
    return 2;
  }
  const prm::Point2D elbow = prm::elbowPosition(arm, t1);
  std::printf("  elbow        = (%8.4f, %8.4f) m\n", elbow.x, elbow.y);
  std::printf("  end effector = (%8.4f, %8.4f) m\n", r.end_effector.x, r.end_effector.y);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const prm::ArmParams arm = prm::defaultArm();
  std::printf("Two-link arm: L1 = %.3f m, L2 = %.3f m\n", arm.link1_length_m, arm.link2_length_m);
  std::printf("Limits: joint1 [%.0f, %.0f] deg, joint2 [%.0f, %.0f] deg\n\n",
              prm::radToDeg(arm.joint1.min_rad), prm::radToDeg(arm.joint1.max_rad),
              prm::radToDeg(arm.joint2.min_rad), prm::radToDeg(arm.joint2.max_rad));

  if (argc == 3) {
    double t1 = 0.0, t2 = 0.0;
    if (!parseDouble(argv[1], t1) || !parseDouble(argv[2], t2)) {
      std::fprintf(stderr, "Error: angles must be numbers (degrees).\n");
      return 1;
    }
    return report(arm, t1, t2);
  }
  if (argc != 1) {
    std::fprintf(stderr, "Usage: %s [theta1_deg theta2_deg]\n", argv[0]);
    return 1;
  }

  std::string a, b;
  while (true) {
    std::cout << "Enter theta1 theta2 in degrees (q to quit): " << std::flush;
    if (!(std::cin >> a) || a == "q") break;
    if (!(std::cin >> b)) break;
    double t1 = 0.0, t2 = 0.0;
    if (!parseDouble(a, t1) || !parseDouble(b, t2)) {
      std::cout << "  Not numbers, try again.\n";
      continue;
    }
    report(arm, t1, t2);
    std::cout << '\n';
  }
  return 0;
}
