#include "definitions.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut piston('h');
pros::adi::DigitalOut piston2('g');
pros::MotorGroup left_mg({-18, -17, -13});
pros::MotorGroup right_mg({15, 19, 9});
//pros::MotorGroup drivetrain({-18, -17, -13, 15, 19, 9});
//pros::Motor roller(8);
pros::Motor lift(21);
pros::Motor Redriect1(11);
pros::Motor Redriect2(-1);
pros::MotorGroup Redriect({11, -1});
pros::Rotation encoder(2);
bool pistonState = false;
bool reversedSteering = false;