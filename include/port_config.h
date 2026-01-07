#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

// Centralized hardware port assignments 
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
#define INTAKE3_MOTOR_PORT   PORT11

// Sensors
#define INERTIAL_SENSOR_PORT PORT20
#define OPTICAL_SENSOR_PORT  PORT1
#define DISTANCE_SENSOR_PORT PORT1
#define HORIZONTAL_TRACKER_PORT Brain.ThreeWirePort.C
#define VERTICAL_TRACKER_PORT   Brain.ThreeWirePort.F

// Triport pneumatics
#define PTO_TRI_PORT    Brain.ThreeWirePort.A
#define TONGUE_TRI_PORT    Brain.ThreeWirePort.B
#define FLAP_PORT   Brain.ThreeWirePort.D
#define SIDE_DESCORE_PORT  Brain.ThreeWirePort.E

#endif  // PORT_CONFIG_H
