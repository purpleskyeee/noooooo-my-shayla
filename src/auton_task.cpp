#include "auton_task.h"

#include "vex.h"
#include "auton_functions.h"
#include "definitions.h"
#include "motor-control.h"
#include "threads.h"
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
  driver_control_active.store(true);
  stopChassis(vex::brakeType::coast);
  heading_correction = false;

  bool lasttonguepressstate = false;
  bool lastbandpressstate = false;
  bool lastdescorepressstate = false;
  bool lastsiddescorestate = false;
  bool lastdefensestate = false;

  while (true) {
    bool intakebutnoscore = controller_1.ButtonR1.pressing();
    bool score = controller_1.ButtonR2.pressing();
    bool tonguemechtoggle = controller_1.ButtonL2.pressing();
    bool outtake = controller_1.ButtonL1.pressing();

    bool rubberbandtoggle = controller_1.ButtonY.pressing();
    bool middescoretoggle = controller_1.ButtonRight.pressing();
    bool sidedescoretoggle = controller_1.ButtonDown.pressing();
    bool defensing = controller_1.ButtonA.pressing();

    axis3 = controller_1.Axis3.position();
    axis1 = controller_1.Axis1.position();
    Right_Power = axis3 - defensechange * axis1;
    Left_Power = axis3 + defensechange * axis1;
    if (Right_Power > 100) Right_Power = 100;
    if (Right_Power < -100) Right_Power = -100;
    if (Left_Power > 100) Left_Power = 100;
    if (Left_Power < -100) Left_Power = -100;

    intake_collect = intakebutnoscore;
    intake_score = score;
    intake_outtake = outtake;

    if (tonguemechtoggle != lasttonguepressstate) {
      tonguemechdown = !tonguemechdown;
    }

    if (rubberbandtoggle != lastbandpressstate && rubberbandtoggle==false) {
      rubberbandon = !rubberbandon;
    }

    if (middescoretoggle != lastdescorepressstate && middescoretoggle==false) {
      middescoreon = !middescoreon;
    }

    if (sidedescoretoggle != lastsiddescorestate && sidedescoretoggle==false) {
      descoreup = !descoreup;
    }

    if (defensing != lastdefensestate) {
      defensechange *= -1;
    }

    lasttonguepressstate = tonguemechtoggle;
    lastbandpressstate = rubberbandtoggle;
    lastdescorepressstate = middescoretoggle;
    lastsiddescorestate = sidedescoretoggle;
    lastdefensestate = defensing;

    vex::this_thread::sleep_for(10);
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
