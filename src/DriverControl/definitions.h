#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"


extern pros::Controller master;
extern pros::adi::DigitalOut piston;
extern pros::adi::DigitalOut doinker;
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::MotorGroup drivetrain;
extern pros::MotorGroup lift;
extern pros::Motor Intake1;
extern pros::Motor Intake2;
extern pros::Motor Redriect1;
extern pros::Motor Redriect2;
extern pros::Motor Redriect;
extern pros::Rotation encoder;
extern pros::Rotation horztracking;
extern pros::MotorGroup drivetrain;


extern bool pistonState;
extern bool doinkerstate;
extern bool reversedSteering;


extern lemlib::Drivetrain yah;

extern lemlib::OdomSensors sensors;


// forward/backward PID
extern lemlib::ControllerSettings lateralController;

// turning PID
extern lemlib::ControllerSettings angularController;

extern lemlib::Chassis chassis;


#endif // DEFINITIONS_H