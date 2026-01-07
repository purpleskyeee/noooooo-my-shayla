#include "threads.h"

#include "vex.h"
#include "definitions.h"
#include "ball_indexer.h"
#include <iostream>

void stopDrive() {
  left_chassis.stop(vex::brakeType::coast);
  right_chassis.stop(vex::brakeType::coast);
}

void stopIntake() {
  intake2Motor.stop(vex::brakeType::coast);
  intake1Motor.stop(vex::brakeType::coast);
  intake3Motor.stop(vex::brakeType::coast);
}

int drivetrain_thread() {
  while (true) {
    left_chassis.spin(vex::directionType::fwd, Left_Power*100, vex::voltageUnits::mV);
    right_chassis.spin(vex::directionType::fwd, Right_Power*100, vex::voltageUnits::mV);
    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int intake_thread() {
  while (true) 
  {

    if(MidGoalScoring)
    {
        flap.set(false);
        intake1Motor.spin(vex::directionType::rev, MAX_VEL, vex::voltageUnits::mV);
        intake2Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
        intake3Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
    }
    else if(LongGoalScoring)
    {
        flap.set(true);
        intake1Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
        intake2Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
        intake3Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
    }
    else if(IntakeCollecting)
    {
        flap.set(false);
        intake1Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
        intake2Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
        intake3Motor.spin(vex::directionType::fwd, MAX_VEL, vex::voltageUnits::mV);
    }
    else if(IntakeOuttaking)
    {  
        flap.set(false);
        intake1Motor.spin(vex::directionType::rev, MAX_VEL, vex::voltageUnits::mV);
        intake2Motor.spin(vex::directionType::rev, MAX_VEL, vex::voltageUnits::mV);
        intake3Motor.spin(vex::directionType::rev, MAX_VEL, vex::voltageUnits::mV);
    }
    else { flap.set(false); stopIntake(); }
    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int pneumatics_thread() {
  while (true) {
    tonguemech.set(TongueState);

    sidedescore.set(DescoreState);

    vex::this_thread::sleep_for(10);
  }
  return 0;
}

int indexer_thread(){
    while(true){
        if(optical_sensor.isNearObject()){
            Ball temp;
            temp.color = optical_sensor.color();
            LongGoal.addBall(temp);
        }
        vex::this_thread::sleep_for(10);
    }
    return 0;
}
