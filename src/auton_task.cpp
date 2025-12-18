#include "auton_task.h"

#include "vex.h"
#include "auton_functions.h"
#include "definitions.h"
#include "motor-control.h"
#include "threads.h"
#include <iostream>
#include "../custom/include/ball-indexer.h"

void AutonSkills() {
  driver_control_active.store(false);
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
  bool driveEngaged = false;
  bool intakeEngaged = true;
  bool ptoEngaged = false;
  bool lastL1 = false;
  bool lastY = false;
  bool hoodEngaged = false;
  bool lastB = false;
  bool midScoreEngaged = true;

  // Set default states
  rubberband.set(midScoreEngaged);
  flap.set(hoodEngaged);

  while (true) {
    // Arcade control scheme
    int dir = controller_1.Axis3.position();
    int turn = controller_1.Axis1.position();
    int left_power = dir - turn;
    int right_power = dir + turn;
    if (left_power > 100) left_power = 100;
    if (left_power < -100) left_power = -100;
    if (right_power > 100) right_power = 100;
    if (right_power < -100) right_power = -100;
    left_chassis.spin(vex::directionType::fwd, left_power * -128 / 100, vex::voltageUnits::mV);
    right_chassis.spin(vex::directionType::fwd, right_power * -128 / 100, vex::voltageUnits::mV);

    if(driveEngaged){
      intake1Motor.spin(vex::directionType::fwd, left_power * -128 / 100, vex::voltageUnits::mV);
      intake2Motor.spin(vex::directionType::fwd, right_power * -128 / 100, vex::voltageUnits::mV);
    } 
    if(intakeEngaged){
      if(controller_1.ButtonR1.pressing()){
        intake1Motor.spin(vex::directionType::fwd, 12000, vex::voltageUnits::mV);
        intake2Motor.spin(vex::directionType::fwd, 12000, vex::voltageUnits::mV);
      } else if(controller_1.ButtonR2.pressing()){
        intake1Motor.spin(vex::directionType::rev, 12000, vex::voltageUnits::mV);
        intake2Motor.spin(vex::directionType::rev, 12000, vex::voltageUnits::mV);
      } else {
        intake1Motor.stop();
        intake2Motor.stop();
      }
    }
    
    // PTO control scheme
    bool currentL1 = controller_1.ButtonL1.pressing();
    if (currentL1 && !lastL1) {
      ptoEngaged = !ptoEngaged;
      pto.set(ptoEngaged);
      driveEngaged = ptoEngaged;
      intakeEngaged = !ptoEngaged;
    }
    lastL1 = currentL1;

    // Tongue mech control
    if (controller_1.ButtonL2.pressing()) {
      tonguemech.set(false);
    } else {
      tonguemech.set(true);
    }

    // Mid goal piston control
    bool currentY = controller_1.ButtonY.pressing();
    if (currentY && !lastY) {
      midScoreEngaged = !midScoreEngaged;
      rubberband.set(midScoreEngaged);
    }
    lastY = currentY;

    // Hood piston control
    bool currentB = controller_1.ButtonB.pressing();
    if (currentB && !lastB) {
      hoodEngaged = !hoodEngaged;
      flap.set(hoodEngaged);
    }
    lastB = currentB;

    // Side descore piston control
    if (controller_1.ButtonRight.pressing()) {
      sidedescore.set(false);
    } else {
      sidedescore.set(true);
    }

    vex::wait(20, vex::msec);
  }
}

void PreAuton() {
  driver_control_active.store(false);
  vexcodeInit();

  inertial_sensor.calibrate();
  while (inertial_sensor.isCalibrating()) {
    vex::this_thread::sleep_for(10);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);

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

  shutdownIndexer();
}
