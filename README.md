# Precision Robotic Manipulation System

Software for controlling a robotic arm to move small objects accurately
between designated positions: a simplified, educational version of the
precision-manipulation tasks found in surgical robotics.

Built in C++ with ROS 2 and MoveIt 2. Developed in stages; this repository
currently contains 

**Phase 1: forward kinematics of a two-link planar arm.**

## Status

| Phase | Goal | Status |
|------:|------|--------|
| 1 | Forward kinematics in C++ with tests | In progress |
| 2 | ROS 2 nodes (publish/subscribe joint commands) | Not started |
| 3 | Control a simulated arm with MoveIt 2 | Not started |
| 4 | Kinematics integration and verification | Not started |
| 5 | Collision-aware motion planning | Not started |
| 6 | Reliability testing and performance report | Not started |
| 7 | Repeatable multi-step precision task (v1.0) | Not started |

## Build and test

Requirements: a C++20 compiler (GCC 11+ or Clang 14+) and CMake 3.22+.
GoogleTest is downloaded automatically if it isn't installed.

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Run the demo:

```bash
./build/fk_cli 30 60     # theta1 = 30 deg, theta2 = 60 deg
./build/fk_cli           # interactive mode
```

## Layout

```
include/prm/two_link_arm.hpp   public API and conventions
src/two_link_arm.cpp           implementation
app/main.cpp                   command-line demo
tests/test_two_link_arm.cpp    unit and property tests
docs/derivation.md             the math, derived step by step
```

## Design decisions

- **Radians internally, degrees only at the user boundary.** 
  Unit mix-upsare a classic source of kinematics bugs.
- **Status codes instead of exceptions on the computation path.** 
  Many medical-device and real-time codebases forbid exceptions; explicit
  statuses force every caller to handle failure.
- **Validation is separate from the math.** `forwardKinematics()` is pure;
  `computeFk()` validates first. The pure function is reused later when
  comparing against MoveIt.
- **Property tests, not just example tests.** Besides hand-checked
  configurations, the tests verify invariants (reachable annulus, link
  length preserved) across a grid of thousands of configurations.

## Learning log

<!-- Add a dated entry each session: what you learned, what confused you,
     what you'd do differently. This becomes the engineering report. -->

## 03/11/26
- Discovering layout of directory and how to structure project
## 06/11/26
- Finished with forward kinematics portion of the project and textbook readings. What's left is to understain main.cpp.

## Discussion 06/11/26

This is part one of journey towards mastering robotics as a craft and creating a robot arm. Four functions were written in order to output the parts of the arm such as the tip or the elbow, and to derive the forward kinematics equation for the arm. Additional functions were implemented to ensure valid inputs to the function, and valid outputs indicating error or status code given the case of an invalid input. Chapters 1-2 from Modern Robotics corresponds to this section of the project, introducing and highlighting the concepts of forward kinematics and answering the question: where is the tip of the robot? 
