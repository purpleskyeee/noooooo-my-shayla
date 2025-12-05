#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <tuple>
#include <atomic>
#include "motor-control.h"
#include "auton_functions.h"
#include "definitions.h"

/*
 * MOTOR CONTROL MODULE MAP
 *  1. Global state + low-level chassis helpers
 *  2. Heading utilities and output scaling helpers
 *  3. Turn/drive primitives (turnToAngle, driveTo, curveCircle, swing, etc.)
 *  4. Sensor-based pose correction utilities
 *  5. Odometry tracking threads
 *  6. Targeting helpers (turnToPoint, moveToPoint, boomerang)
 */

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
bool is_turning = false;
double prev_left_output = 0, prev_right_output = 0;
double x_pos = 0, y_pos = 0;
double correct_angle = 0;

namespace {
std::atomic<bool> intake_thread_should_run{false};
std::atomic<bool> intake_thread_running{false};
std::atomic<double> intake_thread_voltage{12.0};
std::atomic<bool> intake_thread_reverse{false};
vex::thread* intake_thread_handle = nullptr;
std::atomic<bool> tongue_thread_should_run{false};
std::atomic<bool> tongue_thread_running{false};
std::atomic<bool> tongue_thread_close_when_stopped{true};
vex::thread* tongue_thread_handle = nullptr;

double clampIntakeVoltage(double volts) {
  if (volts > 12.0) return 12.0;
  if (volts < 0) return 0.0;
  return volts;
}

int intakeThreadMain() {
  intake_thread_running = true;
  while (intake_thread_should_run.load()) {
    double v = clampIntakeVoltage(fabs(intake_thread_voltage.load()));
    bool reverse = intake_thread_reverse.load();
    vex::directionType hoodDir = vex::directionType::rev;
    vex::directionType intakeDir = reverse ? vex::directionType::rev : vex::directionType::fwd;
    hoodMotor.spin(hoodDir, v, vex::voltageUnits::volt);
    intakeMotor.spin(intakeDir, v, vex::voltageUnits::volt);
    vex::wait(10, vex::msec);
  }
  hoodMotor.stop(vex::brakeType::coast);
  intakeMotor.stop(vex::brakeType::coast);
  intake_thread_running = false;
  return 0;
}

int tongueThreadMain() {
  tongue_thread_running = true;
  tonguemech.open();
  while (tongue_thread_should_run.load()) {
    printf("poopyrunning\n");
    vex::wait(20, vex::msec);
  }
  if (tongue_thread_close_when_stopped.load()) {
    printf("poopyclose\n");
    tonguemech.close();
  }
  tongue_thread_running = false;
  return 0;
}
}

// ============================================================================
// CHASSIS CONTROL FUNCTIONS
// ============================================================================

/*
 * Sets the voltage output for the left and right chassis motors.
 * - left_power: Voltage for the left side (in volts).
 * - right_power: Voltage for the right side (in volts).
 */
void driveChassis(double left_power, double right_power) {
  // Spin left and right chassis motors with specified voltages
  left_chassis.spin(fwd, left_power, voltageUnits::volt);
  right_chassis.spin(fwd, right_power, voltageUnits::volt);
}

/*
 * Stops both chassis motors with the specified brake type.
 * - type: Brake mode (coast, brake, or hold).
 */
void stopChassis(brakeType type) {
  // Stop left and right chassis motors using the given brake type
  left_chassis.stop(type);
  right_chassis.stop(type);
}

/*
 * Resets the rotation position of both chassis motors to zero.
 */
void resetChassis() {
  // Set both chassis motor encoders to zero
  left_chassis.setPosition(0, degrees);
  right_chassis.setPosition(0, degrees);
}

/*
 * Returns the current rotation of the left chassis motor in degrees.
 */
double getLeftRotationDegree() {
  // Get left chassis motor position in degrees
  return left_chassis.position(degrees);
}

/*
 * Returns the current rotation of the right chassis motor in degrees.
 */
double getRightRotationDegree() {
  // Get right chassis motor position in degrees
  return right_chassis.position(degrees);
}

void startIntakeThread(double voltage, bool reverse) {
  double clamped = clampIntakeVoltage(fabs(voltage));
  intake_thread_voltage = clamped;
  intake_thread_reverse = reverse;
  if (intake_thread_should_run.load()) {
    return;
  }
  intake_thread_should_run = true;
  intake_thread_handle = new vex::thread(intakeThreadMain);
  while (!intake_thread_running.load()) {
    vex::wait(5, vex::msec);
  }
}

void stopIntakeThread(vex::brakeType stopType) {
  if (!intake_thread_should_run.load() && intake_thread_handle == nullptr) {
    hoodMotor.stop(stopType);
    intakeMotor.stop(stopType);
    return;
  }
  intake_thread_should_run = false;
  if (intake_thread_handle != nullptr) {
    intake_thread_handle->join();
    delete intake_thread_handle;
    intake_thread_handle = nullptr;
  }
  hoodMotor.stop(stopType);
  intakeMotor.stop(stopType);
}

bool isIntakeThreadRunning() {
  return intake_thread_running.load();
}

void startTongueThread(bool closeWhenStopped) {
  tongue_thread_close_when_stopped = closeWhenStopped;
  printf("poopy\n");
  if (tongue_thread_should_run.load()) {
    printf("poopy1\n");
    return;
  }
  tongue_thread_should_run = true;
  tongue_thread_handle = new vex::thread(tongueThreadMain);
  while (!tongue_thread_running.load()) {
    vex::wait(5, vex::msec);
  }
}

void stopTongueThread() {
  if (!tongue_thread_should_run.load() && tongue_thread_handle == nullptr) {
    if (tongue_thread_close_when_stopped.load()) {
      tonguemech.close();
    }
    return;
  }
  tongue_thread_should_run = false;
  if (tongue_thread_handle != nullptr) {
    printf("poopykill\n");
    tongue_thread_handle->join();
    delete tongue_thread_handle;
    tongue_thread_handle = nullptr;
  }
  if (tongue_thread_close_when_stopped.load()) {
    printf("poopycloseout\n");
    tonguemech.close();
  }
}

bool isTongueThreadRunning() {
  return tongue_thread_running.load();
}

/*
 * Normalizes an angle to be within +/-180 degrees of the current heading.
 * - angle: The target angle to normalize.
 */
double normalizeTarget(double angle) {
  // Adjust angle to be within +/-180 degrees of the inertial sensor's rotation
  if (angle - getInertialHeading() > 180) {
    while (angle - getInertialHeading() > 180) angle -= 360;
  } else if (angle - getInertialHeading() < -180) {
    while (angle - getInertialHeading() < -180) angle += 360;
  }
  return angle;
}

/*
 * Returns the current inertial sensor heading in degrees.
 * - normalize: If true, normalizes the heading (not used in this implementation).
 */
double getInertialHeading() {
  // Get inertial sensor rotation in degrees
  return inertial_sensor.rotation(degrees);
}

// ============================================================================
// OUTPUT SCALING HELPER FUNCTIONS
// ============================================================================

/*
 * Ensures output values are at least the specified minimum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - min_output: Minimum allowed output voltage.
 */
void scaleToMin(double& left_output, double& right_output, double min_output) {
  // Scale outputs to ensure minimum voltage is met for both sides
  if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  } else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}

/*
 * Ensures output values do not exceed the specified maximum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - max_output: Maximum allowed output voltage.
 */
void scaleToMax(double& left_output, double& right_output, double max_output) {
  // Scale outputs to ensure maximum voltage is not exceeded for both sides
  if (fabs(left_output) >= fabs(right_output) && left_output > max_output) {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output > max_output) {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  } else if (fabs(left_output) > fabs(right_output) && left_output < -max_output) {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output < -max_output) {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}

// ============================================================================
// MAIN DRIVE AND TURN FUNCTIONS
// ============================================================================

/*
 * Turns the robot to a specified angle using PID control.
 * - turn_angle: Target angle to turn to (in degrees).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  // Prepare for turn
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  // Normalize and set PID target
  turn_angle = normalizeTarget(turn_angle);
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw baseline for visualization
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // PID loop for turning
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = getInertialHeading();
  double previous_heading = 0;
  int index = 1;
  if(exit == false && correct_angle < turn_angle) {
    // Turn right without stopping at end
    while (getInertialHeading() < turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading); // PID update for heading
      // Draw heading trace
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    // Turn left without stopping at end
    while (getInertialHeading() > turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(-output, output);
      wait(10, msec);
    }
  } else {
    // Standard PID turn
    while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  }
  if(exit) {
    stopChassis(vex::hold);
  }
  correct_angle = turn_angle;
  is_turning = false;
}

// Rotate the robot by a relative angle using the existing turn PID.
// delta_angle_deg: relative degrees to rotate (positive right, negative left).
void rotateBy(double delta_angle_deg, double time_limit_msec, bool exit, double max_output) {
  double target = normalizeTarget(getInertialHeading() + delta_angle_deg);
  turnToAngle(target, time_limit_msec, exit, max_output);
}

void alignToSetHeading(double time_limit_msec, double max_output) {
  double target = normalizeTarget(correct_angle);
  turnToAngle(target, time_limit_msec, true, max_output);
}

// ============================================================================
// SENSOR-BASED POSE CORRECTION
// ============================================================================

namespace {
bool readingsWithinSensorRange(double front_mm, double right_mm) {
  constexpr double minRange = 20.0;
  constexpr double maxRange = 6000.0;
  return front_mm >= minRange && front_mm <= maxRange &&
         right_mm >= minRange && right_mm <= maxRange;
}

bool averageFrontRightDistances(int samples, double& front_avg_mm, double& right_avg_mm) {
  double front_sum = 0;
  double right_sum = 0;
  int valid_samples = 0;
  for (int i = 0; i < samples; ++i) {
    double f = front_distance.objectDistance(mm);
    double r = right_distance.objectDistance(mm);
    if (f > 1 && r > 1) {
      front_sum += f;
      right_sum += r;
      ++valid_samples;
    }
    wait(20, msec);
  }
  if (valid_samples == 0) return false;

  front_avg_mm = front_sum / valid_samples;
  right_avg_mm = right_sum / valid_samples;
  return true;
}
}

// correctPoseFromFrontRightDistances
// Attempts to estimate robot heading and position using two distance sensors:
// one pointing forward and one pointing to the right. The math assumes the
// measured walls are axis-aligned (vertical and horizontal) at world
// coordinates `wall_x_in` and `wall_y_in` (inches). Sensor offsets are given
// in inches in the robot frame: x to the right, y forward.
void correctPoseFromFrontRightDistances(double front_mm, double right_mm,
                                       double fx, double fy,
                                       double rx, double ry,
                                       double wall_x_in, double wall_y_in) {
  const double deg_to_rad = M_PI / 180.0;
  const double mm_to_in = 1.0 / 25.4;
  double Df = front_mm * mm_to_in;
  double Dr = right_mm * mm_to_in;

  double odom_x = x_pos;
  double odom_y = y_pos;
  double initial_theta = getInertialHeading() * deg_to_rad;

  auto implied_pose = [&](double th) {
    double c = cos(th);
    double s = sin(th);
    double rx_wx = c * rx - s * ry;
    double fx_wy = s * fx + c * fy;
    double p_x = wall_x_in - Dr * c - rx_wx;
    double p_y = wall_y_in - Df * c - fx_wy;
    return std::tuple<double,double,double>(p_x, p_y, th);
  };

  auto pose_error = [&](double th)->double {
    auto pose = implied_pose(th);
    double px = std::get<0>(pose);
    double py = std::get<1>(pose);
    double ex = px - odom_x;
    double ey = py - odom_y;
    return ex*ex + ey*ey;
  };

  double best_theta = initial_theta;
  double best_err = pose_error(best_theta);
  for (double d = -45.0; d <= 45.0; d += 1.0) {
    double th = initial_theta + d * deg_to_rad;
    double err = pose_error(th);
    if (err < best_err) { best_err = err; best_theta = th; }
  }

  for (int iter = 0; iter < 3; ++iter) {
    double start = best_theta - 5.0 * deg_to_rad;
    double end = best_theta + 5.0 * deg_to_rad;
    double step = (end - start) / 200.0;
    for (double th = start; th <= end; th += step) {
      double err = pose_error(th);
      if (err < best_err) { best_err = err; best_theta = th; }
    }
  }

  auto best_pose = implied_pose(best_theta);
  double new_x = std::get<0>(best_pose);
  double new_y = std::get<1>(best_pose);

  x_pos = new_x;
  y_pos = new_y;
  correct_angle = fmod((best_theta / deg_to_rad) + 540.0, 360.0) - 180.0;

  Brain.Screen.clearLine();
  Brain.Screen.printAt(10, 20, "Pose corrected: x=%.2f in y=%.2f in th=%.2f deg", x_pos, y_pos, correct_angle);
}

bool autoCorrectFromFrontRightSensors(int samples, double max_apply_shift_in,
                                      double wall_x_override, double wall_y_override) {
  if (max_apply_shift_in < 0) max_apply_shift_in = max_pose_correction_shift_in;
  double front_avg_mm = 0;
  double right_avg_mm = 0;
  if (!averageFrontRightDistances(samples, front_avg_mm, right_avg_mm)) return false;
  if (!readingsWithinSensorRange(front_avg_mm, right_avg_mm)) return false;

  double wall_x_in = std::isnan(wall_x_override) ? reference_wall_x_in : wall_x_override;
  double wall_y_in = std::isnan(wall_y_override) ? reference_wall_y_in : wall_y_override;

  double before_x = x_pos;
  double before_y = y_pos;
  correctPoseFromFrontRightDistances(front_avg_mm, right_avg_mm,
                                     front_distance_offset_x, front_distance_offset_y,
                                     right_distance_offset_x, right_distance_offset_y,
                                     wall_x_in, wall_y_in);

  double dx = x_pos - before_x;
  double dy = y_pos - before_y;
  double shift = sqrt(dx*dx + dy*dy);
  if (shift > max_apply_shift_in) {
    x_pos = before_x;
    y_pos = before_y;
    return false;
  }
  return true;
}

bool captureReferenceWallsFromSensors(int samples) {
  if (samples <= 0) samples = 1;
  double front_avg_mm = 0;
  double right_avg_mm = 0;
  if (!averageFrontRightDistances(samples, front_avg_mm, right_avg_mm)) {
    return false;
  }
  if (!readingsWithinSensorRange(front_avg_mm, right_avg_mm)) {
    return false;
  }

  const double mm_to_in = 1.0 / 25.4;
  double front_in = front_avg_mm * mm_to_in;
  double right_in = right_avg_mm * mm_to_in;

  double heading_rad = degToRad(getInertialHeading());
  double c = cos(heading_rad);
  double s = sin(heading_rad);

  double rx_wx = c * right_distance_offset_x - s * right_distance_offset_y;
  double fx_wy = s * front_distance_offset_x + c * front_distance_offset_y;

  reference_wall_x_in = x_pos + right_in * c + rx_wx;
  reference_wall_y_in = y_pos + front_in * c + fx_wy;

  Brain.Screen.printAt(10, 40, "Wall refs: x=%.2f y=%.2f", reference_wall_x_in, reference_wall_y_in);
  return true;
}

bool correctHeadingFromSensors(int samples, double max_apply_shift_in,
                               double align_time_msec, double align_max_output,
                               double desired_heading_deg,
                               double wall_x_override, double wall_y_override) {
  if (max_apply_shift_in < 0) max_apply_shift_in = max_pose_correction_shift_in;
  if (!autoCorrectFromFrontRightSensors(samples, max_apply_shift_in,
                                        wall_x_override, wall_y_override)) {
    return false;
  }

  if (!std::isnan(desired_heading_deg)) {
    double target = normalizeTarget(desired_heading_deg);
    turnToAngle(target, align_time_msec, true, align_max_output);
    correct_angle = target;
  } else {
    alignToSetHeading((int)align_time_msec, align_max_output);
    correct_angle = normalizeTarget(getInertialHeading());
  }
  return true;
}

/*
 * Drives the robot a specified distance (in inches) using PID control.
 * - distance_in: Target distance to drive (positive or negative).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output, double exit_velocity, bool enforce_heading) {
  // Store initial encoder values
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  distance_in = distance_in * drive_direction;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Configure PID controllers
  pid_distance.setTarget(distance_in);
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(correct_angle));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_distance = 0, current_angle = 0;

  // Main PID loop for driving straight
  while (((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec)) {
    // Calculate current distance and heading
    current_distance = (fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in) + fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in)) / 2;
    current_angle = getInertialHeading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = pid_heading.update(current_angle);

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
    if(!exit) {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    left_output += correction_output;
    right_output -= correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit) {
    if(fabs(exit_velocity) > 1e-6) {
      double v = exit_velocity;
      if(v > max_output) v = max_output;
      if(v < -max_output) v = -max_output;
      prev_left_output = v;
      prev_right_output = v;
      driveChassis(v, v);
    } else {
      prev_left_output = 0;
      prev_right_output = 0;
      stopChassis(vex::hold);
    }
  }
  is_turning = false;
}

/*
 * CurveCircle
 * Drives the robot in a circular arc with a specified radius and angle.
 * - result_angle_deg: Target ending angle (in degrees) for the arc.
 * - center_radius: Radius of the circle's center (positive for curve to the right, negative for curve to the left).
 * - time_limit_msec: Maximum time allowed for the curve (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  // Store initial encoder values for both sides
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // Normalize the target angle to be within +/-180 degrees of the current heading
  result_angle_deg = normalizeTarget(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;

  // Calculate arc lengths for inner and outer wheels
  in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distance_between_wheels / 2)) * result_angle);
  ratio = in_arc / out_arc;

  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;

  // Determine curve and drive direction
  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  // Slew/min speed handling for chaining
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  double prev_curve_left = 0;
  double prev_curve_right = 0;
  auto enforceSlew = [&](double& left, double& right) {
    if(prev_curve_left - left > max_slew_rev) left = prev_curve_left - max_slew_rev;
    if(prev_curve_right - right > max_slew_rev) right = prev_curve_right - max_slew_rev;
    if(left - prev_curve_left > max_slew_fwd) left = prev_curve_left + max_slew_fwd;
    if(right - prev_curve_right > max_slew_fwd) right = prev_curve_right + max_slew_fwd;
    prev_curve_left = left;
    prev_curve_right = right;
  };

  // Initialize PID controllers for arc distance and heading correction
  PID pid_out = PID(distance_kp, distance_ki, distance_kd);
  PID pid_turn = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_out.setTarget(out_arc);
  pid_out.setIntegralMax(0);  
  pid_out.setIntegralRange(5);
  pid_out.setSmallBigErrorTolerance(0.3, 0.9);
  pid_out.setSmallBigErrorDuration(50, 250);
  pid_out.setDerivativeTolerance(threshold * 4.5);

  pid_turn.setTarget(0);
  pid_turn.setIntegralMax(0);  
  pid_turn.setIntegralRange(1);
  pid_turn.setSmallBigErrorTolerance(0, 0);
  pid_turn.setSmallBigErrorDuration(0, 0);
  pid_turn.setDerivativeTolerance(0);
  pid_turn.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;

  // Main control loop for each curve/exit configuration
  if (curve_direction == -1 && exit == true) {
    // Left curve, stop at end
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      // Calculate the real angle along the arc
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      // Enforce minimum output if chaining
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      // Apply heading correction
      left_output += correction_output;
      right_output -= correction_output;

      // Enforce maximum output
      scaleToMax(left_output, right_output, max_output);
      enforceSlew(left_output, right_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    // Right curve, stop at end
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);
      enforceSlew(left_output, right_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    // Left curve, chaining (do not stop at end)
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }
      
      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);
      enforceSlew(left_output, right_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else {
    // Right curve, chaining (do not stop at end)
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);
      enforceSlew(left_output, right_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  }
  // Stop the chassis if required
  if(exit == true) {
    stopChassis(vex::brakeType::hold);
  }
  // Update the global heading
  correct_angle = result_angle_deg;
  is_turning = false;
}

/*
 * Swing
 * Performs a swing turn, rotating the robot around a point while driving forward or backward.
 * - swing_angle: Target angle to swing to (in degrees).
 * - drive_direction: Direction to drive (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the swing (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  stopChassis(vex::brakeType::coast); // Stop chassis before starting swing
  is_turning = true;                  // Set turning state
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd); // Initialize PID for turning

  swing_angle = normalizeTarget(swing_angle); // Normalize target angle
  pid.setTarget(swing_angle);                 // Set PID target
  pid.setIntegralMax(0);  
  pid.setIntegralRange(5);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline for visualization
  double draw_amplifier = 230 / fabs(swing_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(swing_angle) * draw_amplifier, 600, fabs(swing_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // Start the PID loop
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = correct_angle;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;

  // Determine which side to swing and direction
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }

  // Swing logic for each case, chaining (exit == false)
  if(choice == 1 && exit == false) {
    // Swing left, forward
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(hold); // Hold left, swing right
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  } else if(choice == 2 && exit == false) {
    // Swing right, forward
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold); // Hold right, swing left
      wait(10, msec);
    }
  } else if(choice == 3 && exit == false) {
    // Swing left, backward
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      wait(10, msec);
    }
  } else {
    // Swing right, backward
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec && exit == false) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  }

  // PID loop for exit == true (stop at end)
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && exit == true) {
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Draw heading trace
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    // Clamp output
    if(output > max_output) output = max_output;
    else if(output < -max_output) output = -max_output;

    // Apply output to correct side based on swing direction
    switch(choice) {
    case 1:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, -output * drive_direction, volt);
      break;
    case 2:
      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 3:
      left_chassis.spin(fwd, -output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 4:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      break;
    }
    wait(10, msec);
  }
  if(exit == true) {
    stopChassis(vex::hold); // Stop chassis at end if required
  }
  correct_angle = swing_angle; // Update global heading
  is_turning = false;          // Reset turning state
}

/*
 * heading_correction
 * Continuously adjusts the robot's heading to maintain a straight course.
 * Uses a PID controller to minimize the error between the current heading and the target heading.
 */
void correctHeading() {
  double output = 0;
  PID pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid.setTarget(correct_angle); // Set PID target to current heading
  pid.setIntegralRange(fabs(correct_angle) / 2.5);

  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);

  // Continuously correct heading while enabled
  while(heading_correction) {
    pid.setTarget(correct_angle);
    if(is_turning == false) {
      output = pid.update(getInertialHeading());
      driveChassis(output, -output); // Apply correction to chassis
    }
    wait(10, msec);
  }
}

/*
 * trackNoOdomWheel
 * Tracks the robot's position using only drivetrain encoders and inertial sensor.
 * Assumes no external odometry tracking wheels.
 */
void trackNoOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_left_deg = 0, prev_right_deg = 0;
  double delta_local_y_in = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double left_deg = getLeftRotationDegree();
    double right_deg = getRightRotationDegree();
    double delta_heading_rad = heading_rad - prev_heading_rad; // Change in heading (radians)
    double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;   // Left wheel delta (inches)
    double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // Right wheel delta (inches)
    // If no heading change, treat as straight movement
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
    } else {
      // Calculate arc movement for each wheel
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
      double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
      delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
    }
    // Update global position using polar coordinates
    double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
    double polar_radius_in = delta_local_y_in;

    x_pos += polar_radius_in * sin(polar_angle_rad);
    y_pos += polar_radius_in * cos(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;

    wait(10, msec);
  }
}

/*
 * trackXYOdomWheel
 * Tracks the robot’s position using both horizontal and vertical odometry wheels plus inertial heading.
 */
void trackXYOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_horizontal_pos_deg = 0, prev_vertical_pos_deg = 0;
  double delta_local_x_in = 0, delta_local_y_in = 0;
  double local_polar_angle_rad = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double horizontal_pos_deg = horizontal_tracker.position(degrees);
    double vertical_pos_deg = vertical_tracker.position(degrees);
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_horizontal_in = (horizontal_pos_deg - prev_horizontal_pos_deg) * horizontal_tracker_diameter * M_PI / 360.0; // horizontal tracker delta (inches)
    double delta_vertical_in = (vertical_pos_deg - prev_vertical_pos_deg) * vertical_tracker_diameter * M_PI / 360.0; // vertical tracker delta (inches)

    // Calculate local movement based on heading change
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_x_in = delta_horizontal_in;
      delta_local_y_in = delta_vertical_in;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_x_in = sin_multiplier * ((delta_horizontal_in / delta_heading_rad) + horizontal_tracker_dist_from_center);
      delta_local_y_in = sin_multiplier * ((delta_vertical_in / delta_heading_rad) + vertical_tracker_dist_from_center);
    }

    // Avoid undefined atan2(0, 0)
    if (fabs(delta_local_x_in) < 1e-6 && fabs(delta_local_y_in) < 1e-6) {
      local_polar_angle_rad = 0;
    } else {
      local_polar_angle_rad = atan2(delta_local_y_in, delta_local_x_in);
    }
    double polar_radius_in = sqrt(pow(delta_local_x_in, 2) + pow(delta_local_y_in, 2));
    double polar_angle_rad = local_polar_angle_rad - heading_rad - (delta_heading_rad / 2);

    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_horizontal_pos_deg = horizontal_pos_deg;
    prev_vertical_pos_deg = vertical_pos_deg;

    wait(10, msec);
  }
}

/*
 * trackXOdomWheel
 * Tracks position using only horizontal odometry wheel + drivetrain encoders + inertial heading.
 */
void trackXOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_horizontal_pos_deg = 0;
  double prev_left_deg = 0, prev_right_deg = 0;
  double delta_local_x_in = 0, delta_local_y_in = 0;
  double local_polar_angle_rad = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double horizontal_pos_deg = horizontal_tracker.position(degrees);
    double left_deg = getLeftRotationDegree();
    double right_deg = getRightRotationDegree();
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_horizontal_in = (horizontal_pos_deg - prev_horizontal_pos_deg) * horizontal_tracker_diameter * M_PI / 360.0; // horizontal tracker delta (inches)
    double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;   // Left wheel delta (inches)
    double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // Right wheel delta (inches)

    // Calculate local movement based on heading change
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_x_in = delta_horizontal_in;
      delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_x_in = sin_multiplier * ((delta_horizontal_in / delta_heading_rad) + horizontal_tracker_dist_from_center);
      double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
      double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
      delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
    }

    if (fabs(delta_local_x_in) < 1e-6 && fabs(delta_local_y_in) < 1e-6) {
      local_polar_angle_rad = 0;
    } else {
      local_polar_angle_rad = atan2(delta_local_y_in, delta_local_x_in);
    }
    double polar_radius_in = sqrt(pow(delta_local_x_in, 2) + pow(delta_local_y_in, 2));
    double polar_angle_rad = local_polar_angle_rad - heading_rad - (delta_heading_rad / 2);

    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);
    
    prev_heading_rad = heading_rad;
    prev_horizontal_pos_deg = horizontal_pos_deg;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;

    wait(10, msec);
  }
}

/*
 * trackYOdomWheel
 * Tracks position using only vertical odometry wheel + inertial heading.
 */
void trackYOdomWheel() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_vertical_pos_deg = 0;
  double delta_local_y_in = 0;

  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double vertical_pos_deg = vertical_tracker.position(degrees);
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_vertical_in = (vertical_pos_deg - prev_vertical_pos_deg) * vertical_tracker_diameter * M_PI / 360.0; // vertical tracker delta (inches)

    // Calculate local movement based on heading change
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_y_in = delta_vertical_in;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_y_in = sin_multiplier * ((delta_vertical_in / delta_heading_rad) + vertical_tracker_dist_from_center);
    }

    double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
    double polar_radius_in = delta_local_y_in;

    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_vertical_pos_deg = vertical_pos_deg;

    wait(10, msec);
  }
}

/*
 * turnToPoint
 * Turns the robot to face a specific point in the field.
 * - x, y: Coordinates of the target point.
 * - direction: Direction to face the point (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 */
void turnToPoint(double x, double y, int direction, double time_limit_msec) {
  stopChassis(vex::brakeType::coast); // Stop chassis before turning
  is_turning = true;                  // Set turning state
  double threshold = 1, add = 0;
  if(direction == -1) {
    add = 180; // Add 180 degrees if turning to face backward
  }
  // Calculate target angle using atan2 and normalize
  double turn_angle = normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))) + add;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  pid.setTarget(turn_angle); // Set PID target
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline for visualization
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // Start the PID loop
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
    // Continuously update target as robot moves
    pid.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))) + add);
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Draw heading trace
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    driveChassis(output, -output); // Apply output to chassis
    wait(10, msec);
  }  
  stopChassis(vex::hold); // Stop at end
  correct_angle = getInertialHeading(); // Update global heading
  is_turning = false;                   // Reset turning state
}

/*
 * moveToPoint
 * Moves the robot to a specific point in the field, adjusting heading as needed.
 * - x, y: Coordinates of the target point.
 * - dir: Direction to move in (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 * - overturn: If true, allows overturning for sharp turns.
 */
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast); // Stop chassis before moving
  is_turning = true;                  // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Set PID targets for distance and heading
  pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
  pid_distance.setIntegralMax(0);  
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);
  
  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  // Main PID loop for moving to point
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    // Continuously update targets as robot moves
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
    pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
    current_angle = getInertialHeading();
    // Calculate drive output based on heading and distance
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x - x_pos, y - y_pos) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (x_pos - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Only apply heading correction if far from target
    if(hypot(x - x_pos, y - y_pos) > 8 && ch == true) {
      correction_output = pid_heading.update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    wait(10, msec);
  }
  if(exit == true) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold); // Stop at end if required
  }
  correct_angle = getInertialHeading(); // Update global heading
  is_turning = false;                   // Reset turning state
}

void wallRideToFrontDistance(double ride_heading_deg,
                             double ride_wall_target_in,
                             double ride_exit_front_in,
                             double ride_max_output,
                             double ride_time_limit_msec,
                             double exit_heading_deg,
                             double drive_front_target_in,
                             double drive_max_output,
                             double drive_time_limit_msec) {
  const double mm_to_in = 1.0 / 25.4;
  auto readFrontIn = [&]() -> double {
    double dist_mm = front_distance.objectDistance(distanceUnits::mm);
    if (dist_mm <= 1) return NAN;
    return dist_mm * mm_to_in;
  };
  auto readRightIn = [&]() -> double {
    double dist_mm = right_distance.objectDistance(distanceUnits::mm);
    if (dist_mm <= 1) return NAN;
    return dist_mm * mm_to_in;
  };

  stopChassis(vex::brakeType::coast);
  PID ride_heading_pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);
  ride_heading_pid.setTarget(normalizeTarget(ride_heading_deg));
  ride_heading_pid.setIntegralMax(0);
  ride_heading_pid.setIntegralRange(1);
  ride_heading_pid.setArrive(false);

  constexpr double wall_follow_kp = 1.5; // volts per inch of wall error
  double ride_start = Brain.timer(msec);
  while (Brain.timer(msec) - ride_start <= ride_time_limit_msec) {
    double front_in = readFrontIn();
    if (!std::isnan(front_in) && front_in <= ride_exit_front_in) {
      break;
    }

    double heading_correction = ride_heading_pid.update(getInertialHeading());
    double wall_in = readRightIn();
    double wall_correction = 0;
    if (!std::isnan(wall_in)) {
      wall_correction = (wall_in - ride_wall_target_in) * wall_follow_kp;
    }

    double left = ride_max_output - heading_correction - wall_correction;
    double right = ride_max_output + heading_correction + wall_correction;
    scaleToMax(left, right, ride_max_output);
    driveChassis(left, right);
    wait(10, msec);
  }
  stopChassis(vex::brakeType::brake);

  turnToAngle(normalizeTarget(exit_heading_deg), 1500, true, ride_max_output);

  PID drive_heading_pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);
  drive_heading_pid.setTarget(normalizeTarget(exit_heading_deg));
  drive_heading_pid.setIntegralMax(0);
  drive_heading_pid.setIntegralRange(1);
  drive_heading_pid.setArrive(false);

  PID front_pid = PID(distance_kp, distance_ki, distance_kd);
  front_pid.setTarget(drive_front_target_in);
  front_pid.setIntegralMax(2);
  front_pid.setSmallBigErrorTolerance(0.25, 1.5);
  front_pid.setSmallBigErrorDuration(50, 200);
  front_pid.setDerivativeTolerance(5);
  front_pid.setArrive(true);

  double drive_start = Brain.timer(msec);
  while (Brain.timer(msec) - drive_start <= drive_time_limit_msec) {
    double front_in = readFrontIn();
    if (std::isnan(front_in)) {
      driveChassis(drive_max_output, drive_max_output);
      wait(10, msec);
      continue;
    }
    double forward_cmd = front_pid.update(front_in);
    forward_cmd = -forward_cmd; // Move forward when current distance is greater than target
    if (forward_cmd > drive_max_output) forward_cmd = drive_max_output;
    if (forward_cmd < -drive_max_output) forward_cmd = -drive_max_output;

    double heading_correction = drive_heading_pid.update(getInertialHeading());
    double left = forward_cmd - heading_correction;
    double right = forward_cmd + heading_correction;
    scaleToMax(left, right, drive_max_output);
    driveChassis(left, right);

    if (front_pid.targetArrived()) {
      break;
    }
    wait(10, msec);
  }
  stopChassis(vex::hold);
  correct_angle = normalizeTarget(exit_heading_deg);
}

/*
 * boomerang
 * Drives the robot in a boomerang-shaped path to a target point.
 * - x, y: Coordinates of the target point.
 * - a: Final angle of the robot to target (in degrees).
 * - dlead: Distance to lead the target by (in inches, set higher for curvier path, don't set above 0.6).
 * - time_limit_msec: Maximum time allowed for the maneuver (in milliseconds).
 * - dir: Direction to move in (1 for forward, -1 for backward).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 * - overturn: If true, allows overturning for sharp turns.
 */
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast); // Stop chassis before moving
  is_turning = true;                  // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_distance.setTarget(0); // Target is dynamically updated
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // Main PID loop for boomerang path
  while ((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    hypotenuse = hypot(x_pos - x, y_pos - y); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - x_pos, carrot_y - y_pos) * dir);
    current_angle = getInertialHeading();
    // Calculate drive output based on carrot point
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - x_pos, carrot_y - y_pos) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(a))) <= (x_pos - x) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Heading correction logic based on distance to carrot/target
    if(hypot(carrot_x - x_pos, carrot_y - y_pos) > 8) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(carrot_x - x_pos, carrot_y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else if(hypot(x - x_pos, y - y_pos) > 6) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if(exit && hypot(x - x_pos, y - y_pos) < 5) {
        break;
      }
    }

    // Limit slip speed for smoother curves
    slip_speed = sqrt(chase_power * getRadius(x_pos, y_pos, carrot_x, carrot_y, current_angle) * 9.8);
    if(left_output > slip_speed) {
      left_output = slip_speed;
    } else if(left_output < -slip_speed) {
      left_output = -slip_speed;
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold); // Stop at end if required
  }
  correct_angle = getInertialHeading(); // Update global heading
  is_turning = false;                   // Reset turning state
}

// ============================================================================
// TEMPLATE NOTE
// ============================================================================
// This file is intended as a template for VEX/V5 robotics teams.
// All functions and variables use clear, consistent naming conventions.
// Comments are concise and explain the intent of each section.
// Teams can adapt PID values, drive base geometry, and logic as needed for their robot.
