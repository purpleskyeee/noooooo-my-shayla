#include "threads.h"

#include "vex.h"
#include "definitions.h"
#include <iostream>

namespace {
constexpr double kDriveMvScale = 128.0;
constexpr double kMaxIntakePowerMv = 12800.0;

void stopDrive() {
  left_chassis.stop(vex::brakeType::coast);
  right_chassis.stop(vex::brakeType::coast);
}

void stopIntake() {
  intake2Motor.stop(vex::brakeType::coast);
  intake1Motor.stop(vex::brakeType::coast);
}
}

int drivetrain_thread() {
  bool was_active = false;
  while (true) {
    left_chassis.spin(vex::directionType::fwd, Left_Power*128, vex::voltageUnits::mV);
    right_chassis.spin(vex::directionType::fwd, Right_Power*128, vex::voltageUnits::mV);

    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int intake_thread() {
  while (true) 
  {
    // std::cout<<"sigmer\n"<<std::endl;
    // std::cout<<"Drive Engaged: "<<driveEngaged;
    // std::cout<<" Intake Engaged: "<<intakeEngaged<<std::endl;
    if(driveEngaged){
			intake1Motor.spin(vex::directionType::fwd, Left_Power*128,vex::voltageUnits::mV);
			intake2Motor.spin(vex::directionType::fwd, Right_Power*128,vex::voltageUnits::mV);
		} 
    if(intakeEngaged){
      if (intake_in) {
        intake2Motor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
        intake1Motor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
      } else if (intake_outtake) {
        // printf("outtaking\n");
        flapdown = false;
        intake2Motor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
        intake1Motor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
      } else {
        stopIntake();
      }
    }
    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int pneumatics_thread() {
  while (true) {
    // std::cout<<"sigmer"<<std::endl;
    std::cout<<tonguemechdown<<" "<<rubberbandon<<" "<<descoreup<<" "<<flapdown<<" "<<ptoengaged<<std::endl;
    tonguemech.set(!tonguemechdown);

    rubberband.set(rubberbandon);

    sidedescore.set(descoreup);

    flap.set(!flapdown);

    pto.set(ptoengaged);

    vex::this_thread::sleep_for(10);
  }
  return 0;
}
