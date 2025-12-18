#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

// Centralized hardware port assignments modeled after the HighStakes template.
// Update the macros below if the robot wiring changes.

// Drive motors
#define LEFT_CHASSIS1_PORT PORT4
#define LEFT_CHASSIS2_PORT PORT6
#define LEFT_CHASSIS3_PORT PORT8
#define RIGHT_CHASSIS1_PORT PORT3
#define RIGHT_CHASSIS2_PORT PORT5
#define RIGHT_CHASSIS3_PORT PORT7

// Subsystems
#define INTAKE1_MOTOR_PORT PORT10
#define INTAKE2_MOTOR_PORT   PORT9
#define ARM_MOTOR1_PORT   PORT1
#define ARM_MOTOR2_PORT   PORT1
#define STAGER_MOTOR_PORT PORT1

// Sensors
#define INERTIAL_SENSOR_PORT PORT20
#define OPTICAL_SENSOR_PORT  PORT1
#define DISTANCE_SENSOR_PORT PORT1
#define HORIZONTAL_TRACKER_PORT PORT1
#define VERTICAL_TRACKER_PORT   PORT1

// Triport pneumatics
#define PTO_TRI_PORT    Brain.ThreeWirePort.A
#define TONGUE_TRI_PORT    Brain.ThreeWirePort.B
#define RUBBER_BAND_PORT   Brain.ThreeWirePort.C
#define FLAP_PORT   Brain.ThreeWirePort.D
#define SIDE_DESCORE_PORT  Brain.ThreeWirePort.E
#define CLAW_PORT          Brain.ThreeWirePort.F
#define RUSH_ARM_PORT      Brain.ThreeWirePort.G
#define MOGO_MECH_PORT     Brain.ThreeWirePort.H

#endif  // PORT_CONFIG_H
