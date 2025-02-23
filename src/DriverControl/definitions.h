#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

extern pros::Controller master;
extern pros::adi::DigitalOut piston;
extern pros::adi::DigitalOut piston2;
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::MotorGroup drivetrain;
extern pros::MotorGroup lift;
extern pros::Motor Intake1;
extern pros::Motor Redriect1;
extern pros::Motor Redriect2;
extern pros::MotorGroup Redriect;
extern pros::Rotation encoder;
extern pros::Rotation horztracking;
extern pros::MotorGroup drivetrain;


extern bool pistonState;
extern bool reversedSteering;

#endif // DEFINITIONS_H