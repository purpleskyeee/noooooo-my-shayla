#ifndef AUTON_FUNCTIONS_H
#define AUTON_FUNCTIONS_H

#include "vex.h"

void IntakeThread();
void runIntake(double voltage,
               double stallDeltaDeg = 1.0,
               int reverseDurationMs = 120,
               double reverseVoltage = -6.0,
               vex::brakeType stopMode = vex::brakeType::coast);
void spinIntakePair(double intakeVolt, double hoodVolt);
void startIntakeCollect(double voltage = 12.0);
void startIntakeScore(double voltage = 12.0);
void startIntakeOuttake(double voltage = 12.0);
void stopIntakeRoller(vex::brakeType stopMode = vex::brakeType::coast);
void startIndexerIntake();
void startIndexerOuttake();
void stopIndexer();
void shutdownIndexer();

#endif  // AUTON_FUNCTIONS_H
