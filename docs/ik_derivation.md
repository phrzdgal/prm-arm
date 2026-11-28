# Inverse kinematics of a two-link planar arm

Week 1 asked: *given the joint angles, where is the tip?*
Week 2 asks the reverse: *I want the tip at (x, y). What angles get it there?*

That reverse question is how real robots are commanded. A surgeon says
"put the instrument here," not "turn joint 2 by 37 degrees."

## 1. Is the target reachable at all?

The tip's distance from the base is `r = sqrt(x² + y²)`.

- Arm fully straight: the tip is `L1 + L2` away. That's the farthest it reaches.
- Arm fully folded: the tip is `|L1 - L2|` away. That's the closest it gets.

So a target is reachable only if `|L1 - L2| <= r <= L1 + L2`. This is the
ring you already tested in week 1, now used as a check.

## 2. Find theta2 with the law of cosines

Draw the triangle **base, elbow, tip**. Its three sides are `L1`, `L2`, and `r`.
The law of cosines connects them to the elbow angle:

```
r² = L1² + L2² + 2·L1·L2·cos(theta2)
```

Solve for `cos(theta2)` and call it `D`:

```
D = (x² + y² - L1² - L2²) / (2·L1·L2)
```

Then `theta2 = ±acos(D)`.

**The ± is the key idea of the week.** Cosine can't tell `+theta` from
`-theta`, so there are two answers: the elbow can bend either way
(`ElbowConfig::Down` uses +, `ElbowConfig::Up` uses -). Both put the tip
on the same target. Drag the arm in the week-1 explorer to (0°, 90°) and
then (90°, -90°) to see it.

## 3. Find theta1

Two angles, subtracted:

```
theta1 = atan2(y, x) - atan2(L2·sin(theta2), L1 + L2·cos(theta2))
```

- `atan2(y, x)` is the direction from the base straight to the target.
- The second term is how far link 1 has to swing *off* that line so that,
  after the elbow bends, the tip lands on target.

**Why `atan2` and not `atan(y / x)`?** `atan(y/x)` can't tell (1, 1) from
(-1, -1), because `y/x` is 1 for both. `atan2` looks at the signs of x and y
separately, so it always returns the right quadrant. It also doesn't break
when `x = 0`.

## 4. Wrap theta1

Subtracting two angles can give something like 200°, which is the same
pose as -160° (the torus again). Wrap it into [-180°, 180°) before
checking joint limits, or you'll reject a pose that's actually fine.

## 5. Check the joint limits

A target can be geometrically reachable and still be off-limits, for
example behind the base, where the shoulder would have to turn past 90°.
That's a different failure from "too far," so it gets its own status.

## Questions to answer in your own words (write them below)

1. Why are there usually two solutions? When is there exactly one?
   There is exactly one soution at the furthest limits the arm can reach, which occur when the shoulder joint points at -90 degrees and +90 degrees. In all other regions in the arm's workspace, there can be two solutions (this is due to elbow-up and elbow-down effect).
   
2. What would go wrong if you skipped clamping `D` into [-1, 1]?
   Rounding errors may provide that D may be less than -1 or greater than +1, which cannot be equated to a cosine function (the range for cosine is between -1 and 1). So, values close to the boundary of the range values must be clamped/rounded up or down to avoid errors in calculating both theta1 and theta2.

3. Why is "out of reach" a different error from "violates joint limits"?
   Why might a robot operator care about the difference?
   "Out of reach" infers thats the target is too far or too close to reach regardless of the arm's pose. "Violates joint limits" infers that the target is close enough, but there doesn't exist a pose that can reach it in any physical way. A robot operator would care about the difference when trying to fix the problem in order to reach the arm's target. A correct fix requires a correct diagnosis of the problem. 

4. A real surgical robot has more than two joints. What happens to the
   number of solutions as joints are added? (Guess; Chapter 6 confirms.)
   Fewer joints than target numbers indicates there isn't a solution at all. If the joints ar equal to the target numbers, there are usually a small, limited set of solutions (two solutions for the arm for example). Finally, more joints than target numbers indicates there are infinitely many solutions as the arm has spare freedom (hold the tip still, and you can still mvoe the elbows around)

## Book

*Modern Robotics*, Chapter 6 introduction: the 2R planar arm. It's the same
problem, with the same two solutions, in the book's notation.
