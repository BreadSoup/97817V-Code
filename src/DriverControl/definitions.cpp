#include "definitions.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut piston('h');
pros::adi::DigitalOut doinker('b');
pros::MotorGroup left_mg({-18, -17, -13});
pros::MotorGroup right_mg({15, 19, 9});
//pros::MotorGroup drivetrain({-18, -17, -13, 15, 19, 9});
//pros::Motor roller(8);
pros::MotorGroup lift({-21,-1});
pros::Motor Redriect1(12);
pros::Motor Redriect2(-0);
pros::Motor Intake1(-1);
pros::Motor Intake2(-21);
pros::Motor Redriect(12); //idk what port 0 does
pros::Rotation encoder(2);
pros::Rotation horztracking(6);
pros::MotorGroup drivetrain({-18,-17,-13,15,19,9});
bool pistonState = false;
bool doinkerstate = false;
bool reversedSteering = false;








#include "lemlib/chassis/trackingWheel.hpp"
 
 
#include "pros/rtos.h"
#include "lemlib/api.hpp" 
 
pros::Imu imu(8); 
pros::MotorGroup leftm({-18, -17, -13});
pros::MotorGroup rightm({15, 19, 9});
lemlib::TrackingWheel horztrackingwheel(&horztracking, lemlib::Omniwheel::NEW_275, -1.5);
// ---------------------- LemLib Setup ----------------------

lemlib::Drivetrain yah(&leftm, // left motor group
                              &rightm, // right motor group
                              12.5,
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);
lemlib::ControllerSettings linearController(10.25, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)              // was 20 
);

// angular motion controller
lemlib::ControllerSettings angularController(1.85, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             14.5, // derivative gain (kD // was 10)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, 
                            nullptr, 
                            &horztrackingwheel,
                            nullptr, 
                            &imu // inertial sensor
);



lemlib::Chassis chassis(yah, linearController, angularController, sensors);

