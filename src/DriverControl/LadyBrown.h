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
const double integral_limit = 0.0;  // Prevent integral wind-up

// PID Variables
double rotation_position = 0.0;
double previous_error = 0.0;
double integral = 0.0;
int target_position = 10;
bool manual_control = false;
bool notjam = false;


// Motor, Encoder, and Controller Instances
extern pros::Motor Redriect1;    
extern pros::Motor Redriect2;    
extern pros::Motor Redriect; 
extern pros::Controller master;

// Macro states
int UP = 140;
int HOVER = 90;
int MID = 50;
int DOWN = 10;

int deadzonelower = 5;
int deadzoneupper = 15;

enum State {
    STATE_UP,
    STATE_MID,
    STATE_DOWN,
    STATE_UPDOWN,
    STATE_HOVER,
    STATE_MANUAL
};

State state = STATE_DOWN; // Added state variable
State prevstate = state;




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

inline void moveback() {
        lift.move(-12000);
        pros::delay(35);
        lift.move(0);

    
}
static int extrapower = 0; 

inline void updateStateFromInput() {
    

    

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) { 
        state = STATE_MID;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        prevstate = state;
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

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {

        extrapower = -3000;


       
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        extrapower = 3000;
        
      }
      else {

        extrapower =0;
        
      }

      
    //   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    //     if (notjam == true) {
    //         notjam = false;
    //     } else {
    //         notjam = true;
    //     }
    // }
    





    
    }
      

    
    



inline void RedirectControl() {

//while (true) {
    static bool ranjam = false;
    static bool nomoveback = false;

    

    // Fetch encoder position
    double rotation_position = encoder.get_position()/100;

    pros::lcd::set_text(5, "Rotation: " + std::to_string(rotation_position));
    pros::lcd::set_text(6, "Target: " + std::to_string(target_position));
    if (state != STATE_UP) {
        ranjam = false;
    }
    if (state != STATE_UPDOWN) {
        nomoveback = false;
    }

    switch (state) {
        case STATE_UP:
             if (ranjam == false && !notjam){
                jamRing();
                ranjam = true;
             }
            target_position = UP;       
            break;
        case STATE_MID:
                target_position = MID;
            
            break;
        case STATE_DOWN:
                target_position = DOWN;
            
            break;

            case STATE_MANUAL: {
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    Redriect.move_voltage(-3000);
                    target_position = rotation_position;
                } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                    Redriect.move_voltage(3000);
                    target_position = rotation_position;
                } else {
                    Redriect.move_voltage(0);

                    //target_position = rotation_position;
                }

                previous_error = 0;
                integral = 0;

                break;
           //     return;
            }
            case STATE_UPDOWN:
            if (nomoveback == false && !nomoveback){
                moveback();
                nomoveback = true;
             }
            
           // moveback();
                target_position = UP;
                break;

         

            // case STATE_HOVER:
            //     jamRing();
            //     target_position = HOVER;
            //     break;
                
        default:
            state = STATE_DOWN;
            break;

            
    }

        

            

        Redriect.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        // PID Control
        double error = target_position - rotation_position;
        integral += error;


        double derivative = error - previous_error;
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        if (state == STATE_DOWN && (rotation_position >= deadzonelower && rotation_position <= deadzoneupper)) { 
            output = 0;
        }

        Redriect.move_voltage(output + extrapower);  // Apply the output voltage
        previous_error = error;

}
//}