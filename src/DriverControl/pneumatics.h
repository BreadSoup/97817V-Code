#include "pros/misc.h"
#ifndef  definitions
#include "definitions.h"
#endif
inline void clampSolenoid(){
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pistonState = !pistonState; // Toggle the state of the piston
            piston.set_value(pistonState); // Activate or deactivate the solenoid
        }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            doinkerstate = !doinkerstate; // Toggle the state of the piston
            doinker.set_value(doinkerstate); // Activate or deactivate the solenoid
        }
    /*if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            piston2.set_value(false); // Activate or deactivate the solenoid
        }*/
}
inline void intializePneumatics(){
   // piston2.set_value(true);
}