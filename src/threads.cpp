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
    bool active = driver_control_active.load();
    if (active) {
      double left_mv = kDriveMvScale * defensechange * Left_Power;
      double right_mv = kDriveMvScale * defensechange * Right_Power;
      left_chassis.spin(vex::directionType::fwd, left_mv, vex::voltageUnits::mV);
      right_chassis.spin(vex::directionType::fwd, right_mv, vex::voltageUnits::mV);
    } else if (was_active) {
      stopDrive();
    }
    was_active = active;
    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int intake_thread() {
  bool was_active = false;
  while (true) {
    bool active = driver_control_active.load();
    if (active) {
      if (intake_collect) {
        hoodMotor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
        intakeMotor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
      } else if (intake_score) {
        hoodMotor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
        intakeMotor.spin(vex::directionType::fwd, kMaxIntakePowerMv, vex::voltageUnits::mV);
      } else if (intake_outtake) {
        hoodMotor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
        intakeMotor.spin(vex::directionType::rev, kMaxIntakePowerMv, vex::voltageUnits::mV);
      } else {
        stopIntake();
      }
    } else if (was_active) {
      stopIntake();
    }
    was_active = active;
    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int pneumatics_thread() {
  while (true) {
    if (driver_control_active.load()) {
      if (tonguemechdown) {
        tonguemech.close();
      } else {
        tonguemech.open();
      }

      if (rubberbandon) {
        rubberband.close();
      } else {
        rubberband.open();
      }

      if (middescoreon) {
        middescore.open();
      } else {
        middescore.close();
      }

      if (descoreup) {
        sidedescore.open();
      } else {
        sidedescore.close();
      }
    }
    vex::this_thread::sleep_for(10);
  }
  return 0;
}
