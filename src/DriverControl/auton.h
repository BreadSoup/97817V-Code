#ifndef AUTON_H
#define AUTON_H

#include "definitions.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

inline void auton(){
    piston.set_value(false);
    drivetrain.move(-50);
    pros::c::delay(950);
    drivetrain.move(0);
    //piston2.set_value(false);
    pros::c::delay(200);
    piston.set_value(true);
    pros::c::delay(1500);
    lift.move(127);
    pros::c::delay(1000);
    Redriect.move_voltage(-100);
    right_mg.move_voltage(12000);
    left_mg.move_voltage(-12000);
    pros::delay(500);
    right_mg.move_voltage(0);
    left_mg.move_voltage(0);
    drivetrain.move(40);
    //pros:pros::c::delay(1000);
    Redriect.move_voltage(0);
    //right_mg.move_voltage(127);
    pros::delay(720);
    drivetrain.move_voltage(0);
    lift.move_voltage(30);
}

#endif