#include "auton_functions.h"

#include "vex.h"
#include "motor-control.h"
#include "utils.h"
#include "pid.h"
#include "definitions.h"

#include <ctime>
#include <cmath>
#include <thread>

#include "../custom/include/ball-indexer.h"

static double clampVolt(double v) {
  if (v > 12.0) return 12.0;
  if (v < -12.0) return -12.0;
  return v;
}

void spinIntakePair(double intakeVolt, double hoodVolt) {
  intakeMotor.spin(intakeVolt >= 0 ? vex::directionType::fwd : vex::directionType::rev,
                   fabs(intakeVolt), vex::voltageUnits::volt);
  hoodMotor.spin(hoodVolt >= 0 ? vex::directionType::fwd : vex::directionType::rev,
                 fabs(hoodVolt), vex::voltageUnits::volt);
}

void startIntakeCollect(double voltage) {
  double v = clampVolt(voltage);
  spinIntakePair(v, -v);
}

void startIntakeScore(double voltage) {
  double v = clampVolt(voltage);
  spinIntakePair(v, v);
}

void startIntakeOuttake(double voltage) {
  double v = clampVolt(voltage);
  spinIntakePair(-v, -v);
}

void stopIntakeRoller(vex::brakeType stopMode) {
  intakeMotor.stop(stopMode);
  hoodMotor.stop(stopMode);
}

void startIndexerIntake() {
  ball_indexer::resetCounts();
  ball_indexer::enable();
}

void startIndexerOuttake() {
  ball_indexer::startOuttake();
}

void stopIndexer() {
  ball_indexer::disable();
}

void shutdownIndexer() {
  ball_indexer::shutdown();
}
