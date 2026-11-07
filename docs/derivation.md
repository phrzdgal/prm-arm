# Forward kinematics of a two-link planar arm

Work this on paper before writing any code. The goal is that you could
re-derive it on a whiteboard in an interview without notes.

## 1. The setup

```
              tip (x, y)
              *
             /
        L2  /   ) theta2  (measured from link 1's direction)
           /
  elbow   o - - - - - - -   (dashed line = where link 1 would continue)
         /
    L1  /
       /  ) theta1  (measured from the base x-axis)
  ----o----------------------> x
     base (0, 0)
```

- Two revolute joints, so the arm has **2 degrees of freedom**.
- Its **configuration space** (Modern Robotics, Ch. 2) is the pair
  `(theta1, theta2)`. Each joint is a circle, so the full C-space is a
  torus, which joint limits cut down to a rectangle.
- Its **task space** is the plane, `(x, y)`.

Forward kinematics is the map from C-space to task space.

## 2. Step 1: the elbow

Link 1 has length L1 and points along angle theta1. Basic trigonometry:

```
x_elbow = L1 cos(theta1)
y_elbow = L1 sin(theta1)
```

## 3. Step 2: the absolute direction of link 2

This is where most people slip. theta2 is measured **relative to link 1**,
so the direction link 2 points in, measured from the base x-axis, is

```
phi = theta1 + theta2
```

Check it: if theta2 = 0, link 2 is a straight continuation of link 1,
so it should point along theta1. It does.

## 4. Step 3: add link 2

```
x = L1 cos(theta1) + L2 cos(theta1 + theta2)
y = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

That's the whole forward-kinematics solution for this arm.

## 5. Sanity checks to do by hand (these become the unit tests)

With L1 = L2 = 1:

| theta1 | theta2 | Predict (x, y) before checking |
|-------:|-------:|-------------------------------|
| 0      | 0      |          (2, 0)               |
| 90     | 0      |          (1, 1)               |
| 0      | 90     |          (1, 1)               |
| 90     | 90     |          (0, 2)               |
| 45     | -45    |           --                  |
| 30     | 180    |           --                  |

Fill in the right-hand column yourself, then compare with the tests.

## 6. Questions to answer in your own words (put them in this file)

1. Why is the tip distance from the base always between |L1 - L2| and
   L1 + L2? What shape is the reachable workspace?

   Those are the bounds of the maximum and minimum length from the base the arm can reach.
   The minimum occurs when theta1 and theta2 are opposite angles/oppose each other by 180 degrees. The maximum occurs when the angles are the same, such that there is a constructive interference. 

2. Most points in the workspace can be reached by two different
   configurations. What are they called, and why? (Hint: "elbow up" and
   "elbow down.") This is the seed of inverse kinematics.

   For a 2-link panar robotic arm, most points in its workspace can be reached by the two configurations "Elbow-up configuration" and "Elbow-down configuration." Inverse kinematics says that given an end-effector position, find the joint angles that produce this output. There can be multiple solutions.

3. What happens at theta2 = 0 or 180 degrees? Why might a controller
   have trouble there? (You'll meet the word "singularity" in Ch. 5.)

   At those angles, the robotic arm reaches a singularity. When the angle is zero, the two links are fully extended in a straight line. When the angle is 180 degrees, link2 faces the opposite direction of link1, so the links fold on each other. A controller might have trouble here due to required joint velocities being extremely large and loss of the ability to move the end effector instantaneously in certain directions. 

4. Why does the code check for NaN before checking joint limits?

   The code checks for NaN before checking joint limits because values that are NaN behave differently from ordinary numbers and can bypass normal comparison logic. 

## 7. Where this is heading

In Modern Robotics Ch. 4 the same idea is written with 4x4 homogeneous
transformation matrices and the product-of-exponentials formula. That
version scales to 6- and 7-joint arms. The two-line formula above is the
special case you can hold in your head.
