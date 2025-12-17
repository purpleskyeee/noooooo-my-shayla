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

// Pre-match setup
void pre_auton(void) {
  PreAuton();
}

// Keep the existing autonomous routine from this project
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

// Driver control loop (local implementation)
void usercontrol(void) {
  constexpr int MAXVELOCITY = 12800;
  // Ensure intake thread is stopped and intakes are idle before driver loop
  INTAKE.interrupt();
  PNEUMATICS.interrupt();
  hoodMotor.stop();
  intakeMotor.stop();
  intake_collect = false;
  intake_score = false;
  intake_outtake = false;

  // Keep rubberband passively extended (vented) at start of driver control
  rubberbandon = false;
  rubberband.open();

  // Default Port D piston retracted (closed) so it starts down
  portd_on = true;
  portd_piston.close();

  // Keep tongue mech passively retracted at start of driver control
  tonguemechdown = false;
  tonguemech.open();

  stopChassis(coast);
  heading_correction = false;
  while (true) {
    // Read controller axes
    int ch1 = controller_1.Axis1.value();
    int ch2 = controller_1.Axis2.value();
    int ch3 = controller_1.Axis3.value();
    int ch4 = controller_1.Axis4.value();

    // Buttons
    bool l1 = controller_1.ButtonL1.pressing();
    bool l2 = controller_1.ButtonL2.pressing();
    bool r1 = controller_1.ButtonR1.pressing();
    bool r2 = controller_1.ButtonR2.pressing();
    bool button_y = controller_1.ButtonY.pressing();
    bool button_right = controller_1.ButtonRight.pressing();
    bool button_down = controller_1.ButtonDown.pressing();
    bool button_up = controller_1.ButtonUp.pressing();
    bool button_a = controller_1.ButtonA.pressing();

    bool intakebutnoscore = r1;
    bool score = r2;
    bool tonguemechtoggle = l2;
    bool outtake = l1;

    bool rubberbandtoggle = button_y;
    bool middescoretoggle = button_up;        // mid descore on up button
    bool sidedescoretoggle = button_right;    // side descore moved to right button to avoid overlap
    bool portdtoggle = button_down;           // Port D piston on down button
    bool defensing = button_a;

    axis3 = controller_1.Axis3.position();
    axis1 = controller_1.Axis1.position();
    Right_Power = axis3 - defensechange * axis1;
    Left_Power = axis3 + defensechange * axis1;
    if (Right_Power > 128) Right_Power = 128;
    if (Right_Power < -128) Right_Power = -128;
    if (Left_Power > 128) Left_Power = 128;
    if (Left_Power < -128) Left_Power = -128;

    if (intakebutnoscore) {
      hoodMotor.spin(vex::directionType::rev, MAXVELOCITY, vex::voltageUnits::mV);
      intakeMotor.spin(vex::directionType::fwd, MAXVELOCITY, vex::voltageUnits::mV);
    } else if (score) {
      hoodMotor.spin(vex::directionType::fwd, MAXVELOCITY, vex::voltageUnits::mV);
      intakeMotor.spin(vex::directionType::fwd, MAXVELOCITY, vex::voltageUnits::mV);
    } else if (outtake) {
      hoodMotor.spin(vex::directionType::rev, MAXVELOCITY, vex::voltageUnits::mV);
      intakeMotor.spin(vex::directionType::rev, MAXVELOCITY, vex::voltageUnits::mV);
    } else {
      hoodMotor.stop();
      intakeMotor.stop();
    }

    if (tonguemechtoggle != lasttonguepressstate) {
      tonguemechdown = !tonguemechdown;
    }

    if (rubberbandtoggle && !lastbandpressstate) {
      rubberbandon = !rubberbandon;
    }

    if (middescoretoggle && !lastdescorepressstate) {
      middescoreon = !middescoreon;
    }

    if (sidedescoretoggle && !lastsiddescorestate) {
      descoreup = !descoreup;
    }

    if (portdtoggle && !lastportdpressstate) {
      portd_on = !portd_on;
    }

    if (portdtoggle && !lastportdpressstate) {
      portd_on = !portd_on;
    }

    if (defensing != lastdefensestate) {
      defensechange *= -1;
    }

    lasttonguepressstate = tonguemechtoggle;
    if (tonguemechdown) tonguemech.close();
    else tonguemech.open();

    lastbandpressstate = rubberbandtoggle;
    if (rubberbandon) rubberband.close();
    else rubberband.open();

    lastdescorepressstate = middescoretoggle;
    if (middescoreon) middescore.open();
    else middescore.close();

    lastsiddescorestate = sidedescoretoggle;
    if (descoreup) sidedescore.open();
    else sidedescore.close();

    lastportdpressstate = portdtoggle;
    if (portd_on) portd_piston.close();
    else portd_piston.open();

    lastportdpressstate = portdtoggle;
    if (portd_on) portd_piston.close();
    else portd_piston.open();

    left_chassis.spin(vex::directionType::fwd, 128 * defensechange * Left_Power, vex::voltageUnits::mV);
    right_chassis.spin(vex::directionType::fwd, 128 * defensechange * Right_Power, vex::voltageUnits::mV);

    wait(10, msec);
  }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while (true) {
    wait(20, msec);
  }
}
