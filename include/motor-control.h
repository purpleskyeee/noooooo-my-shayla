#include <string>
#include <cmath>

// --- Global Variables (snake_case) ---
extern bool is_turning;

extern double xpos, ypos;
extern double correct_angle;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);

// Turn to an absolute heading (degrees).
void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
// Rotate the robot by a relative angle (degrees) using the turn PID.
// Positive delta turns to the right, negative to the left.
void rotateBy(double delta_angle_deg, double time_limit_msec, bool exit = true, double max_output = 12);
// driveTo(distance_in, time_limit_msec, exit, max_output, exit_velocity, enforce_heading)
// - If `enforce_heading` is true, `driveTo` will pre-align the robot to
//   `correct_angle` before driving and (when `exit==true`) re-align to
//   `correct_angle` afterwards. This is optional and default false.
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12, double exit_velocity = 0, bool enforce_heading = false);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
// Use two distance sensors (front and right) to correct pose.
// - `front_mm` and `right_mm` are sensor readings in millimeters.
// - sensor offsets (`fx, fy`, `rx, ry`) are sensor positions relative to robot center in inches (robot frame: x right, y forward).
// - `wall_x_in` and `wall_y_in` are the world coordinates of the nearest walls (in inches) the sensors measure to (use 0 for corner walls).
// This function estimates heading and position and updates `xpos`, `ypos`, and `correct_angle`.
void correctPoseFromFrontRightDistances(double front_mm, double right_mm,
									   double fx, double fy,
									   double rx, double ry,
									   double wall_x_in = 0.0, double wall_y_in = 0.0);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);

void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void wallRideToFrontDistance(double ride_heading_deg,
						 double ride_wall_target_in,
						 double ride_exit_front_in,
						 double ride_max_output,
						 double ride_time_limit_msec,
						 double exit_heading_deg,
						 double drive_front_target_in,
						 double drive_max_output,
						 double drive_time_limit_msec);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
// Align to the currently stored `correct_angle` (previous heading set).
// Calls `turnToAngle(normalizeTarget(correct_angle), ...)`.
void alignToSetHeading(double time_limit_msec = 800, double max_output = 12);
// Read front/right distance sensors, average samples, validate range, and
// call `correctPoseFromFrontRightDistances` using the geometry constants set
// in robot-config. Returns true if correction was applied. `max_apply_shift_in`
// prevents applying corrections that jump the robot more than this many
// inches from current odometry. Pass a negative value (default) to use the
// configured `max_pose_correction_shift_in`.
bool autoCorrectFromFrontRightSensors(int samples = 5, double max_apply_shift_in = -1.0,
									 double wall_x_in = NAN, double wall_y_in = NAN);
bool captureReferenceWallsFromSensors(int samples = 6);

// Convenience wrapper: perform auto-correction from front/right sensors and
// then align to the stored `correct_angle` (or a supplied heading) if correction succeeded.
bool correctHeadingFromSensors(int samples = 5, double max_apply_shift_in = -1.0,
					double align_time_msec = 800, double align_max_output = 12.0,
					double desired_heading_deg = NAN,
					double wall_x_in = NAN, double wall_y_in = NAN);

void startIntakeThread(double voltage, bool reverse);
void stopIntakeThread(vex::brakeType stopType = vex::brakeType::coast);
bool isIntakeThreadRunning();
void startTongueThread(bool closeWhenStopped = false);
void stopTongueThread();
bool isTongueThreadRunning();