#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>

#include "port_config.h"

using namespace vex;

// Device forward declarations (mirrors the original robot-config header)
extern brain Brain;

extern controller controller_1;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor left_chassis3;
extern motor_group left_chassis;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor right_chassis3;
extern motor_group right_chassis;
extern inertial inertial_sensor;
extern optical example_optical_sensor;
extern distance example_distance_sensor;
extern digital_out example_piston;
extern rotation horizontal_tracker;
extern rotation vertical_tracker;
extern motor intakeMotor;
extern motor hoodMotor;
extern pneumatics tonguemech;
extern pneumatics rubberband;
extern pneumatics middescore;
extern pneumatics sidedescore;

extern motor arm_motor1;
extern motor arm_motor2;
extern motor_group arm_motor;
extern motor intake_motor;
extern motor &intake_primary_motor;
extern motor &intake_stage1_motor;
extern motor &intake_stage2_motor;
extern digital_out claw;
extern digital_out rush_arm;
extern optical optical_sensor;
extern distance intake_distance;
extern distance clamp_distance;
extern distance storage_distance_sensor;
extern distance middle_goal_scoring_sensor;
extern distance long_goal_scoring_sensor;
extern distance &top_distance_sensor;
extern distance &front_distance;
extern distance &right_distance;
extern digital_out mogo_mech;

// Sensor geometry and correction parameters
extern double front_distance_offset_x;
extern double front_distance_offset_y;
extern double right_distance_offset_x;
extern double right_distance_offset_y;
extern double reference_wall_x_in;
extern double reference_wall_y_in;
extern double max_pose_correction_shift_in;

// Motion tuning values
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;

extern bool using_horizontal_tracker;
extern bool using_vertical_tracker;
extern double horizontal_tracker_dist_from_center;
extern double vertical_tracker_dist_from_center;
extern double horizontal_tracker_diameter;
extern double vertical_tracker_diameter;

extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;

// Driver-control shared state (used by the refactored threads)
extern double axis3;
extern double axis1;
extern double Right_Power;
extern double Left_Power;
extern bool tonguemechdown;
extern bool rubberbandon;
extern bool middescoreon;
extern bool descoreup;
extern int defensechange;
extern bool intake_collect;
extern bool intake_score;
extern bool intake_outtake;
extern std::atomic<bool> driver_control_active;

// Lifecycle hooks
void vexcodeInit(void);

#endif  // DEFINITIONS_H
