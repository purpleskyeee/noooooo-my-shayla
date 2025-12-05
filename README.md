# Autonomous Movement Reference

This document shows how to call movement functions implemented in `src/motor-control.cpp` and how to combine them inside `src/auton_task.cpp` routines.

## Movement Primitives

Each section below summarizes what the helper does, how to pick its inputs, and gives an example call that matches the current implementation in `src/motor-control.cpp`.

### `turnToAngle`
- **Signature:** `turnToAngle(double angleDeg, double timeLimitMs, bool exit, double maxVolt, double exitVelocity = 0)`
- **Concept:** PID turn to an absolute inertial heading.
- **Inputs:**
  - `angleDeg`: Absolute goal heading (robot auto-normalizes around current heading).
  - `timeLimitMs`: Safety timeout for the turn loop.
  - `exit`: `true` applies hold or stop at the end; `false` keeps motors spinning for chaining.
  - `maxVolt`: Voltage cap (0–12 V).
  - `exitVelocity`: Optional spin voltage to leave applied when `exit` is true.
- **Example:**
  ```cpp
  turnToAngle(135, 1200, true, 10); // turn to 135° and hold
  ```

### `driveTo`
- **Signature:** `driveTo(double distanceIn, double timeLimitMs, bool exit, double maxVolt, double exitVelocity = 0, bool enforceHeading = true)`
- **Concept:** Linear PID that drives a set distance while correcting heading.
- **Inputs:**
  - `distanceIn`: Inches to travel (positive forward, negative backward).
  - `timeLimitMs`: Timeout for the move.
  - `exit`: `false` for chaining (stays in coast/drive), `true` to stop/hold.
  - `maxVolt`: Voltage cap.
  - `exitVelocity`: Optional constant drive voltage on exit.
  - `enforceHeading`: Leave `true` to maintain `correct_angle`; set `false` if you purposely want to bias heading.
- **Example:**
  ```cpp
  driveTo(48, 2000, true, 11); // drive forward 4 ft and hold
  ```

### `curveCircle`
- **Signature:** `curveCircle(double resultAngleDeg, double centerRadiusIn, double timeLimitMs, bool exit, double maxVolt)`
- **Concept:** Maintains a fixed turning radius by coordinating inner/outer wheel distances while PID tracking heading along the arc.
- **Inputs:**
  - `resultAngleDeg`: Absolute heading when the arc finishes.
  - `centerRadiusIn`: Radius from robot center to circle center (positive = curve right, negative = curve left). Use values greater than half the track width.
  - `timeLimitMs`, `exit`, `maxVolt`: Same semantics as other moves.
- **Example:**
  ```cpp
  curveCircle(0, 20, 1800, true, 9); // 20" right arc back to heading 0°
  ```

### `swing`
- **Signature:** `swing(double targetAngleDeg, double driveDir, double timeLimitMs, bool exit, double maxVolt)`
- **Concept:** Spins only one side of the drivetrain so the robot pivots around the other side.
- **Inputs:**
  - `targetAngleDeg`: Absolute heading goal.
  - `driveDir`: `1` for forward-side swing, `-1` for reverse-side swing.
  - `timeLimitMs`, `exit`, `maxVolt`: Standard behavior.
- **Example:**
  ```cpp
  swing(90, 1, 900, true, 8); // swing forward side to face 90°
  ```

### `moveToPoint`
- **Signature:** `moveToPoint(double x, double y, int dir, double timeLimitMs, bool exit, double maxVolt, bool overturn = false)`
- **Concept:** Chases a target coordinate by continuously updating both distance and heading targets.
- **Inputs:**
  - `x`, `y`: Target world coordinates (inches) relative to `x_pos`/`y_pos`.
  - `dir`: `1` to face and drive forward, `-1` to drive backwards while facing the point.
  - `timeLimitMs`, `exit`, `maxVolt`: Same semantics as other moves.
  - `overturn`: Allow heading correction to clamp drive side for sharp turns.
- **Example:**
  ```cpp
  moveToPoint(36, 60, 1, 2500, true, 10); // forward to (36,60)
  ```

### `boomerang`
- **Signature:** `boomerang(double x, double y, int dir, double finalAngleDeg, double dlead, double timeLimitMs, bool exit, double maxVolt, bool overturn)`
- **Concept:** “Carrot chase” controller—creates a lookahead carrot to blend distance + heading smoothly, finishing at `finalAngleDeg`.
- **Inputs:**
  - `x`, `y`, `dir`, `timeLimitMs`, `exit`, `maxVolt`, `overturn`: Same meaning as `moveToPoint`.
  - `finalAngleDeg`: Desired heading when reaching `(x,y)`.
  - `dlead`: Lookahead multiplier (0–0.6). Higher = smoother but wider path.
- **Example:**
  ```cpp
  boomerang(24, 72, 1, 135, 0.35, 3200, true, 10, true);
  ```

### `correctHeadingFromSensors`
- **Signature:** `correctHeadingFromSensors(int samples, double maxApplyShift, double alignTimeMs, double alignMaxVolt, double desiredHeadingDeg = NAN)`
- **Concept:** Samples the configured front/right distance sensors, snaps odometry (`x_pos`, `y_pos`, `correct_angle`) to the reference walls, then optionally re-aligns heading.
- **Inputs:**
  - `samples`: Number of readings to average per sensor.
  - `maxApplyShift`: Rejects pose corrections exceeding this distance; pass a negative value to use `max_pose_correction_shift_in` from `definitions.cpp`.
  - `alignTimeMs`, `alignMaxVolt`: Limits for the follow-up heading alignment.
  - `desiredHeadingDeg`: If `NAN`, align to `correct_angle`; otherwise align to the specified heading.
- **Example:**
  ```cpp
  bool ok = correctHeadingFromSensors(8, -1.0, 900, 8, 90);
  ```

## Complete Examples

```cpp
void rushStack() {
  driveTo(48, 1800, false, 12);        // keep rolling
  driveTo(12, 800, true, 8);           // final brake
  turnToAngle(0, 1000, true, 8);       // snapshot heading
}

void alignAgainstWall() {
  if (correctHeadingFromSensors(10, 2.5, 800, 8, 90)) {
    driveTo(6, 600, true, 6);
  }
}

void collectCornerRing() {
  task passiveIntake([] { while (true) runIntake(8); });
  boomerang(36, 60, 1, 135, 0.4, 3000, true, 10, true);
  passiveIntake.stop();
  holdIntake();
}
```

Use these patterns as building blocks inside `custom/src/autonomous.cpp` to keep autonomous routines concise and readable. Tune PID gains and geometry constants in `src/definitions.cpp` to match your robot.
