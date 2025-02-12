#include "definitions.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/llemu.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>

// PID Constants (Tuned for better precision and faster response)
const double kP = 190;   // Reduced proportional gain to avoid overshoot
const double kI = 0;  // Reduced integral gain to prevent wind-up
const double kD = 100.0;   // Increased derivative gain to smooth out the response
const double gravity_comp = 000.0;  // Gravity compensation
const double integral_limit = 0.0;  // Prevent integral wind-up

// PID Variables
double previous_error = 0.0;
double integral = 0.0;
int target_position = 10;
bool manual_control = false;
bool moving_to_600 = false;
bool moving_back_to_0 = false;

// Motor, Encoder, and Controller Instances
extern pros::Motor Redriect1;    
extern pros::Motor Redriect2;    
extern pros::MotorGroup Redriect; 
extern pros::Controller master;

// Quad Encoder Ports
#define QUAD_TOP_PORT 'F'
#define QUAD_BOTTOM_PORT 'E'

// Initialize Shaft Encoder
//pros:: encoder(QUAD_TOP_PORT, QUAD_BOTTOM_PORT, false);
inline void RedirectControl() {

    // Fetch encoder position
    double encoder_position = encoder.get_position()/100;
    //double encoder_position+100;
    

    // Display encoder position
    pros::lcd::set_text(2, "Encoder: " + std::to_string(encoder_position));
    pros::lcd::set_text(3, "Target: " + std::to_string(target_position));

    // Manual control via controller buttons
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        Redriect.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        Redriect.move_voltage(-3000);  // Max voltage for faster movement
        manual_control = true;
        target_position = encoder_position;
    } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        Redriect.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        Redriect.move_voltage(3000);  // Max voltage for faster movement
        manual_control = true;
        target_position = encoder_position;
    } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        Redriect.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Redriect.move(0);
        manual_control = false;
    } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        // Setting target position when LEFT button is pressed
        target_position = 42;
        manual_control = false;
        moving_to_600 = false;
        moving_back_to_0 = false;
    } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        // Setting target position when RIGHT button is pressed
        target_position = 140;
        moving_to_600 = true;
    } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        // Setting target position when UP button is pressed
        target_position = 140;
        lift.move(-100);
        lift.move(0);
        moving_back_to_0 = false;
    
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        // Setting target position when DOWN button is pressed
        target_position = 10;
        moving_back_to_0 = true;
    
    }
    else {
        // If no manual control, perform PID control
        if (manual_control) {
            target_position = encoder_position;
            manual_control = false;
        }
        else if (moving_to_600 && encoder_position >= 130) {
            target_position = 10;
            moving_to_600 = false;
            moving_back_to_0 = true;
        }
        

        Redriect.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        // PID Control
        double error = target_position - encoder_position;
        integral += error;

        // Limit integral to prevent wind-up
        integral = std::clamp(integral, -integral_limit, integral_limit);

        double derivative = error - previous_error;
        double output = (kP * error) + (kI * integral) + (kD * derivative) + gravity_comp;


        // Increase output range if needed
        output = std::clamp(output, -12700.0, 12700.0);  // Max voltage to the motors
        if (moving_back_to_0 && (encoder_position >= 5 && encoder_position <= 15)) { 
            output = 0;
        }
        Redriect.move_voltage(output);  // Apply the output voltage
        previous_error = error;

        // Stop condition
        
    }
    pros::delay(20);  // Prevents CPU overload and allows time for sensor updates
}
