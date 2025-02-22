#include "definitions.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut piston('h');
pros::adi::DigitalOut piston2('g');
pros::MotorGroup left_mg({-18, -17, -13});
pros::MotorGroup right_mg({15, 19, 9});
//pros::MotorGroup drivetrain({-18, -17, -13, 15, 19, 9});
//pros::Motor roller(8);
pros::MotorGroup lift({-21,-1});
pros::Motor Redriect1(11);
pros::Motor Redriect2(-0);
pros::Motor Intake1(-1);
pros::MotorGroup Redriect({0, -0}); //idk what port 0 does
pros::Rotation encoder(2);
pros::MotorGroup drivetrain({-18,-17,-13,15,19,9});
bool pistonState = false;
bool reversedSteering = false;