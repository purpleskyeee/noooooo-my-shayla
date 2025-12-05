#include "vex.h"

#include <atomic>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
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
optical example_optical_sensor = optical(OPTICAL_SENSOR_PORT);
distance example_distance_sensor = distance(DISTANCE_SENSOR_PORT);
digital_out example_piston = digital_out(Brain.ThreeWirePort.A);
motor intakeMotor = motor(INTAKE_MOTOR_PORT, ratio6_1, false);
motor hoodMotor = motor(HOOD_MOTOR_PORT, ratio6_1, false);
pneumatics tonguemech = pneumatics(TONGUE_TRI_PORT);
pneumatics rubberband = pneumatics(RUBBER_BAND_PORT);
pneumatics middescore = pneumatics(MID_DESCORE_PORT);
pneumatics sidedescore = pneumatics(SIDE_DESCORE_PORT);

rotation horizontal_tracker = rotation(HORIZONTAL_TRACKER_PORT, true);
rotation vertical_tracker = rotation(VERTICAL_TRACKER_PORT, true);

motor arm_motor1 = motor(ARM_MOTOR1_PORT, ratio18_1, true);
motor arm_motor2 = motor(ARM_MOTOR2_PORT, ratio18_1, false);
motor_group arm_motor = motor_group(arm_motor1, arm_motor2);
motor intake_motor = motor(STAGER_MOTOR_PORT, ratio18_1, true);
motor &intake_primary_motor = intake_motor;
motor &intake_stage1_motor = intake_motor;
motor &intake_stage2_motor = intake_motor;
digital_out claw = digital_out(CLAW_PORT);
digital_out rush_arm = digital_out(RUSH_ARM_PORT);
optical optical_sensor = optical(OPTICAL_SENSOR_PORT);
distance intake_distance = distance(DISTANCE_SENSOR_PORT);
distance clamp_distance = distance(DISTANCE_SENSOR_PORT);
distance storage_distance_sensor = distance(DISTANCE_SENSOR_PORT);
distance middle_goal_scoring_sensor = distance(DISTANCE_SENSOR_PORT);
distance long_goal_scoring_sensor = distance(DISTANCE_SENSOR_PORT);
distance &top_distance_sensor = long_goal_scoring_sensor;
distance &front_distance = intake_distance;
distance &right_distance = clamp_distance;
digital_out mogo_mech = digital_out(MOGO_MECH_PORT);

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

// Driver-control shared state defaults
double axis3 = 0;
double axis1 = 0;
double Right_Power = 0;
double Left_Power = 0;
bool tonguemechdown = false;
bool rubberbandon = false;
bool middescoreon = false;
bool descoreup = false;
int defensechange = 1;
bool intake_collect = false;
bool intake_score = false;
bool intake_outtake = false;
std::atomic<bool> driver_control_active{false};

// VEXcode generated functions
bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}
