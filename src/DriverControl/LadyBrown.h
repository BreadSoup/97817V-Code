#include "definitions.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// PID Constants (Tuned for better precision and faster response)
const double kP = 190;   // Reduced proportional gain to avoid overshoot
const double kI = 0;  // Reduced integral gain to prevent wind-up
const double kD = 100.0;   // Increased derivative gain to smooth out the response
const double integralLimit = 0.0;  // Prevent integral wind-up

// PID Variables
double rotationPosition = 0.0;
double previousError = 0.0;
double integral = 0.0;
int targetPosition = 10;
bool notJam = false;


// Motor, Encoder, and Controller Instances
// extern pros::Motor Redriect1;    
// extern pros::Motor Redriect2;    
// extern pros::Motor Redriect; 
// extern pros::Controller master;

// Macro states
int UP = 140;
int HOVER = 90;
int MID = 50;
int DOWN = 10;

int deadZoneLower = 5;
int deadZoneUpper = 15;

enum State {
    STATE_UP,
    STATE_MID,
    STATE_DOWN,
    STATE_UPDOWN,
    STATE_HOVER,
    STATE_MANUAL
};

State state = STATE_DOWN; // Added state variable
State prevState = state;




inline void jamRing() {
    for (int i = 0; i < 2; ++i) {
        lift.move_voltage(12000);
        pros::delay(110);
        lift.move(-12000);
        pros::delay(40);
    }
    lift.move_voltage(12000);
    pros::delay(100);
    lift.move_voltage(-12000);
    pros::delay(30);
    lift.move(0);                                     
}
inline void moveBack() {
        lift.move(-12000);
        pros::delay(35);
        lift.move(0);

    
}
static int extrapower = 0; 

inline void updateStateFromInput() {
    

    

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) { 
        state = STATE_MID;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        prevState = state;
        state = STATE_UP;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        state = STATE_DOWN;        
    }  
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1 )) {
        state = STATE_MANUAL;
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2 )) {
        state = STATE_MANUAL;
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        state = STATE_UPDOWN;
    }
    
    }
      

    
    



inline void RedirectControl() {

//while (true) {
    static bool ranjam = false;
    static bool noMoveBack = false;

    

    // Fetch encoder position
    double rotationPosition = encoder.get_position()/100.0;

    pros::lcd::set_text(5, "Rotation: " + std::to_string(rotationPosition));
    pros::lcd::set_text(6, "Target: " + std::to_string(targetPosition));
    if (state != STATE_UP) {
        ranjam = false;
    }
    if (state != STATE_UPDOWN) {
        noMoveBack = false;
    }

    switch (state) {
        case STATE_UP:
             if (ranjam == false && !notJam){
                jamRing();
                ranjam = true;
             }
            targetPosition = UP;       
            break;
        case STATE_MID:
                targetPosition = MID;
            
            break;
        case STATE_DOWN:
                targetPosition = DOWN;
            
            break;

            case STATE_MANUAL: {
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    Redriect.move_voltage(-3000);
                    targetPosition = rotationPosition;
                } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                    Redriect.move_voltage(3000);
                    targetPosition = rotationPosition;
                } else {
                    Redriect.move_voltage(0);

                    //targetPosition = rotationPosition;
                }

                previousError = 0;
                integral = 0;

                break;
           //     return;
            }
            case STATE_UPDOWN:
            if (noMoveBack == false && !noMoveBack){
                moveBack();
                noMoveBack = true;
             }
            
                targetPosition = UP;
                break;

         

        default:
            state = STATE_DOWN;
            break;

            
    }

        

            

        Redriect.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        // PID Control
        double error = targetPosition - rotationPosition;
        integral += error;


        double derivative = error - previousError;
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        if (state == STATE_DOWN && (rotationPosition >= deadZoneLower && rotationPosition <= deadZoneUpper)) { 
            output = 0;
        }

        Redriect.move_voltage(output);  // Apply the output voltage
        previousError = error;

}
//}