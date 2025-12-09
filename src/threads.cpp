#include "threads.h"

#include "vex.h"
#include "definitions.h"

namespace {
constexpr double kDriveMvScale = 128.0;
constexpr double kMaxIntakePowerMv = 12800.0;

void stopDrive() {
  left_chassis.stop(vex::brakeType::coast);
  right_chassis.stop(vex::brakeType::coast);
}

void stopIntake() {
  hoodMotor.stop(vex::brakeType::coast);
  intakeMotor.stop(vex::brakeType::coast);
}
}

int drivetrain_thread() {
  bool was_active = false;
  while (true) {
    double left_mv = kDriveMvScale * defensechange * Left_Power;
    double right_mv = kDriveMvScale * defensechange * Right_Power;
    left_chassis.spin(vex::directionType::fwd, left_mv, vex::voltageUnits::mV);
    right_chassis.spin(vex::directionType::fwd, right_mv, vex::voltageUnits::mV);

    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int intake_thread() {
  while (true) 
  {
    if (intake_collect) {
      printf("collecting\n");
      hoodMotor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
      intakeMotor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
    } else if (intake_score) {
      printf("scoring\n");
      hoodMotor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
      intakeMotor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
    } else if (intake_outtake) {
      printf("outtaking\n");
      hoodMotor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
      intakeMotor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
    } else {
      stopIntake();
    }
    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int pneumatics_thread() {
  while (true) {
    
    tonguemech.set(!tonguemechdown);

    rubberband.set(rubberbandon);

    middescore.set(middescoreon);

    sidedescore.set(descoreup);

    vex::this_thread::sleep_for(10);
  }
  return 0;
}
