#include "definitions.h"
#include "pros/colors.hpp"
#pragma once
// inline void rollerControl() {
//   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
//     lift.move_voltage(-12000);
//     Intake1.move_voltage(-12000);
//   } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
//     lift.move_voltage(12000);
//     Intake1.move_voltage(12000);
//   }
//   else {
//     lift.move(0);
//     Intake1.move(0);
//   }
// }
#include "definitions.h"
#include "pros/rtos.h"
#include "definitions.h"
#include "pros/rtos.h"

// Define Intake States
enum IntakeState {
    IDLE,
    INTAKING,
    REVERSING,
    SORTING_RED,
    SORTING_BLUE
};

bool intake_on = false;
bool intakerevdone = false;
bool revtop = false;
bool stopbottom = false;
bool kickblue = true;
bool kickred = false;

// Function to handle the intake state machine
void intakeStateMachine(void* param) {
    IntakeState state = IDLE;
    int stateStartTime = 0;
    
    const int normalVoltage = 12000;
    const int reverseVoltage = -12000;
    const int proxThreshold = 200;  // Only run color sorting if proximity sensor reading >= 200

    while (true) {
        int currentTime = pros::millis();
        double hue = colorsensor.get_hue();
        int proxValue = colorsensor.get_proximity(); 

        // **State Transitions**


if (intake_on)
{
  Intake1.move_voltage(12000);
  Intake2.move_voltage(12000);
}
else
{

        switch (state) {
            case IDLE:
                Intake1.move_voltage(0);
                Intake2.move_voltage(0);
              //  lift.move(0);

                // Start intake if button is pressed
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                    state = INTAKING;
                } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                    state = REVERSING;
                    stateStartTime = currentTime;
                }

                // Only check for sorting if proximity sensor indicates an object is present
                if (proxValue >= proxThreshold) {
                    if (((hue >= 1 && hue < 30) || (hue > 330 && hue <= 360)) && kickred) {
                        state = SORTING_RED;
                        stateStartTime = currentTime;
                    } else if (hue >= 150 && hue <= 260 && kickblue) {
                        state = SORTING_BLUE;
                        stateStartTime = currentTime;
                    }
                }
                break;

            case INTAKING:
                Intake1.move_voltage(-reverseVoltage);
                Intake2.move_voltage(normalVoltage);
              //  lift.move_voltage(normalVoltage);

                // Stop intake when button is released
                if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                    state = IDLE;
                }

                // Check for sorting only if the proximity sensor meets the threshold
                if (proxValue >= proxThreshold) {
                    if (((hue >= 1 && hue < 30) || (hue > 330 && hue <= 335)) && kickred) {
                        state = SORTING_RED;
                        stateStartTime = currentTime;
                    } else if (hue >= 150 && hue <= 260 && kickblue) {
                        state = SORTING_BLUE;
                        stateStartTime = currentTime;
                    }
                }
                break;

            case REVERSING:
                Intake1.move_voltage(-normalVoltage);
                Intake2.move_voltage(reverseVoltage);
                if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                    state = IDLE;
                }
                break;

            case SORTING_RED:
                printf("Red detected - Sorting\n");
                // Run full speed for a short burst then stop
                if (currentTime - stateStartTime < 140) {
                    Intake2.move_voltage(normalVoltage);
                } else {
                    Intake2.move_voltage(0);
                }
                // Return to idle after 800 ms
                if (currentTime - stateStartTime >= 800) {
                    state = IDLE;
                }
                break;

            case SORTING_BLUE:
            printf("blue detected - Sorting\n");
            // Run full speed for a short burst then stop
            if (currentTime - stateStartTime < 140) {
                Intake2.move_voltage(normalVoltage);
            } else {
                Intake2.move_voltage(0);
            }
            // Return to idle after 800 ms
            if (currentTime - stateStartTime >= 800) {
                state = IDLE;
            }
            break;
        }}

        // Debugging output
        printf("State: %d, Hue: %lf, Prox: %d\n", state, hue, proxValue);
        pros::delay(20);
    }
}
