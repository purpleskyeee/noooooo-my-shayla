#include "auton_task.h"

#include "vex.h"
#include "definitions.h"
#include "motor-control.h"
#include "threads.h"
#include <iostream>

void AutonSkills() {
  captureReferenceWallsFromSensors(8);
  int auton_selected = 3;
  switch (auton_selected) {
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

void DriverControl() {

  while (true) {
    // Arcade control scheme
    int dir = controller_1.Axis3.position();
    int turn = controller_1.Axis1.position();
    int left_power = dir + turn;
    int right_power = dir - turn;
    if (left_power > 100) left_power = 100;
    if (left_power < -100) left_power = -100;
    if (right_power > 100) right_power = 100;
    if (right_power < -100) right_power = -100;
    left_chassis.spin(vex::directionType::fwd, left_power * 128, vex::voltageUnits::mV);
    right_chassis.spin(vex::directionType::fwd, right_power * 128, vex::voltageUnits::mV);

    double CurrentR1 = controller_1.ButtonR1.pressing();
    double CurrentR2 = controller_1.ButtonR2.pressing();
    double CurrentL1 = controller_1.ButtonL1.pressing();
    if (CurrentR1&&CurrentR2) {
      LongGoalScoring=true;
      MidGoalScoring=false;
      IntakeCollecting=false;
      IntakeOuttaking=false;
    } else if (CurrentR1) {
      LongGoalScoring=false;
      MidGoalScoring=false;
      IntakeCollecting=true;
      IntakeOuttaking=false;
    } else if(CurrentR2) {
      LongGoalScoring=false;
      MidGoalScoring=true;
      IntakeCollecting=false;
      IntakeOuttaking=false;
    } else if(CurrentL1) {
      LongGoalScoring=false;
      MidGoalScoring=false;
      IntakeCollecting=false;
      IntakeOuttaking=true;
    } else {
      LongGoalScoring=false;
      MidGoalScoring=false;
      IntakeCollecting=false;
      IntakeOuttaking=false;
    }

    bool CurrentRight = controller_1.ButtonRight.pressing();
    if(CurrentRight){ TongueState=1; }
    else{ TongueState=0; }

    bool CurrentY = controller_1.ButtonY.pressing();
    bool CurrentL2 = controller_1.ButtonL2.pressing();
    if(CurrentY||CurrentL2){ DescoreState=1; }
    else{ DescoreState=0; }
    vex::wait(20, vex::msec);
  }
}

void PreAuton() {
  vexcodeInit();
  inertial_sensor.calibrate();
  while (inertial_sensor.isCalibrating()) {
    vex::this_thread::sleep_for(10);
  }

  double current_heading = inertial_sensor.heading();
  // Brain.Screen.print(current_heading);

  resetChassis();
  if (using_horizontal_tracker && using_vertical_tracker) {
    vex::thread odom = vex::thread(trackXYOdomWheel);
    (void)odom;
  } else if (using_horizontal_tracker) {
    vex::thread odom = vex::thread(trackXOdomWheel);
    (void)odom;
  } else if (using_vertical_tracker) {
    vex::thread odom = vex::thread(trackYOdomWheel);
    (void)odom;
  } else {
    vex::thread odom = vex::thread(trackNoOdomWheel);
    (void)odom;
  }
}
