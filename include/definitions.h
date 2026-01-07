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

extern motor intake1Motor;
extern motor intake2Motor;
extern motor intake3Motor;
extern pneumatics tonguemech;
extern pneumatics flap;
extern pneumatics sidedescore;

extern optical optical_sensor;
extern distance intake_distance;
extern distance clamp_distance;

extern distance front_distance;
extern distance right_distance;
extern encoder horizontal_tracker;
extern encoder vertical_tracker;

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
extern const double MAX_VEL;

// Driver-control shared state (used by the refactored threads)
extern double axis3;
extern double axis1;
extern double Right_Power;
extern double Left_Power;
extern bool TongueState;
extern bool DescoreState;
extern int DefenseMode;

extern double MidGoalScoring;
extern double LongGoalScoring;
extern double IntakeCollecting;
extern double IntakeOuttaking;

extern double AutonState;

// Lifecycle hooks
void vexcodeInit(void);

#endif  // DEFINITIONS_H
