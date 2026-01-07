#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

controller controller_1 = controller(primary);

motor left_chassis1 = motor(LEFT_CHASSIS1_PORT, ratio6_1, true);
motor left_chassis2 = motor(LEFT_CHASSIS2_PORT, ratio6_1, true);
motor left_chassis3 = motor(LEFT_CHASSIS3_PORT, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(RIGHT_CHASSIS1_PORT, ratio6_1, false);
motor right_chassis2 = motor(RIGHT_CHASSIS2_PORT, ratio6_1, false);
motor right_chassis3 = motor(RIGHT_CHASSIS3_PORT, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

inertial inertial_sensor = inertial(INERTIAL_SENSOR_PORT);

motor intake1Motor = motor(INTAKE1_MOTOR_PORT, ratio6_1, false);
motor intake2Motor = motor(INTAKE2_MOTOR_PORT, ratio6_1, true);
motor intake3Motor = motor(INTAKE3_MOTOR_PORT, ratio6_1, true);

pneumatics tonguemech = pneumatics(TONGUE_TRI_PORT);
pneumatics flap = pneumatics(FLAP_PORT);
pneumatics sidedescore = pneumatics(SIDE_DESCORE_PORT);

optical optical_sensor = optical(OPTICAL_SENSOR_PORT);

distance intake_distance = distance(DISTANCE_SENSOR_PORT);
distance clamp_distance = distance(DISTANCE_SENSOR_PORT);
distance front_distance = distance(DISTANCE_SENSOR_PORT);
distance right_distance = distance(DISTANCE_SENSOR_PORT);
encoder vertical_tracker = encoder(VERTICAL_TRACKER_PORT);
encoder horizontal_tracker = encoder(HORIZONTAL_TRACKER_PORT);

// Default geometry constants for correctAndAlign helpers (update to match robot)
double front_distance_offset_x = 0.0;
double front_distance_offset_y = 6.0;
double right_distance_offset_x = 3.0;
double right_distance_offset_y = 2.0;
double reference_wall_x_in = 0.0;
double reference_wall_y_in = 144.0;
double max_pose_correction_shift_in = 2.5;

// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
double distance_between_wheels = 11.28;
double wheel_distance_in = (36.0 / 48.0) * 3.17 * M_PI;
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;
double turn_kp = 0.3, turn_ki = 0, turn_kd = 2.5;
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4;

bool using_horizontal_tracker = false;
bool using_vertical_tracker = false;
double horizontal_tracker_dist_from_center = 2.71875;
double vertical_tracker_dist_from_center = -0.03125;
double horizontal_tracker_diameter = 1.975;
double vertical_tracker_diameter = 1.975;

bool heading_correction = true;
bool dir_change_start = true;
bool dir_change_end = true;
double min_output = 10;
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;
double chase_power = 2;
const double MAX_VEL = 12800.0;

// Driver-control shared state defaults
double axis3 = 0;
double axis1 = 0;
double Right_Power = 0;
double Left_Power = 0;
bool TongueState = false;
bool DescoreState = true;
int DefenseMode = 1;

double MidGoalScoring = false;
double LongGoalScoring = false;
double IntakeCollecting = false;
double IntakeOuttaking = false;

double AutonState = false; //has auton finished

void vexcodeInit(void) {}
