/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang (1698V)                                      */
/*    Created:      July 9, 2023                                              */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "port_config.h"
#include "definitions.h"
#include "auton_functions.h"
#include "auton_task.h"
#include "threads.h"
#include "pid.h"
#include "motor-control.h"

using namespace vex;

// A global instance of competition
competition Competition;

//thread DRIVETRAIN(drivetrain_thread);
thread INTAKE(intake_thread);
thread PNEUMATICS(pneumatics_thread);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  PreAuton();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*-------------------------------
--------------------------------------------*/

void autonomous(void) {
  rubberbandon = true;
  tonguemechdown = false;
  driveTo(33, 2000, true, 12, 0, true);
  turnToAngle(-45, 2000, true, 12);
  driveTo(1, 500, true, 8, 0, true);
  intake_outtake = true;
  wait(2500, msec);
  intake_outtake = false;
  driveTo(4, 500, true, 8, 0, true);
  driveTo(-48, 2000, true, 12, 0, true);
  turnToAngle(-180, 2000, true, 12);
  tonguemechdown = true;
  wait(500, msec);
  intake_collect = true;
  driveTo(24, 2000, true, 12, 0, true);
  intake_collect = false;
  driveTo(-40, 2000, true, 12, 0, true);
  intake_score = true;
  wait(2000, msec);
  intake_score = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;
int MAXVELOCITY = 12800;
void usercontrol(void) {
 DriverControl();
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  autonomous();
  usercontrol();

  while (true) {
    wait(20, msec);
  }
}
