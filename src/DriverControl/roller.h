#pragma once

#include "definitions.h"
#include "pros/colors.hpp"
#include "definitions.h"
#include "pros/rtos.h"
#include "definitions.h"
#include "pros/rtos.h"
#include "LadyBrown.h"

enum IntakeState {
    IDLE,
    INTAKING,
    REVERSING,
    SORTING_RED,
    SORTING_BLUE
};
enum AutoState {
    IntakeNormal,
    IntakeUnstuck,
    AUTO_SORTING_RED,
    AUTO_SORTING_BLUE,

    LB_AUTO

};

bool intake_on = false;
bool intakerevdone = false;
bool revtop = false;
bool stopbottom = false;
bool kickblue = true;
bool kickred = false;
bool autointake = false;

// bool readyLB = false;

// bool scoreLB = false;
bool putRingInLB = false;
bool ringWaiting = false;

bool LBempty = true;


void intakeStateMachine(void* param) {
    IntakeState state = IDLE;
    static int stateAuto = IntakeNormal;
    int stateStartTime = 0;

   const int velocityThreshold = 10;
    
    const int normalVoltage = 12000;
    const int reverseVoltage = -12000;
    const int proxThreshold = 200; 

    while (true) {
        int currentTime = pros::millis();
        double hue = colorsensor.get_hue();
        int proxValue = colorsensor.get_proximity(); 



        if (autointake){

            // if (!intake_on && !stopbottom) {
            //     Intake2.move_voltage(0);
            //     Intake1.move_voltage(12000);
            // }
            // else if  (!intake_on && stopbottom) {
            //     Intake1.move_voltage(0);
            //     Intake2.move_voltage(0);
            // } 

             if (!intake_on && revtop) {

                Intake1.move_voltage(-12000);
                Intake2.move_voltage(-12000);
            }
            else {
            switch (stateAuto) {
                case IntakeNormal: 
                    Intake1.move_voltage(normalVoltage);
                    Intake2.move_voltage(normalVoltage);


                    if (proxValue >= proxThreshold) {

                        if ((((hue >= 1 && hue < 30) || (hue > 330 && hue <= 335)) && kickred && readyLB) ){
                            stateAuto = LB_AUTO;
                            stateStartTime = currentTime;
                        }


                        else if (((hue >= 1 && hue < 30) || (hue > 330 && hue <= 335)) && kickred) {
                            stateAuto = AUTO_SORTING_RED;
                            stateStartTime = currentTime;
                        } else if (hue >= 150 && hue <= 260 && kickblue) {
                            stateAuto = AUTO_SORTING_BLUE;
                            stateStartTime = currentTime;
                        }
                    }



                    if (Intake2.get_actual_velocity() < velocityThreshold) {
                        if (stateStartTime == 0) {
                            stateStartTime = currentTime;
                        }
                        if (stateStartTime != 0 && (currentTime - stateStartTime >= 1000)) {
                            stateAuto = IntakeUnstuck;
                            stateStartTime = currentTime;
                        }
                    }
                    if (lift.get_actual_velocity() >= velocityThreshold) {
                        stateStartTime = 0;
                    }
                    
                    // if (readyLB){
                    //     stateAuto = LB_AUTO;
                    //     stateStartTime = currentTime;
                    // }

                    break;

                case IntakeUnstuck:
                    Intake2.move_voltage(reverseVoltage);
                    if (currentTime - stateStartTime >= 1600) {
                        stateAuto = IntakeNormal;
                        stateStartTime = currentTime;
                    }
                    break;

                case AUTO_SORTING_RED:
                 //   printf("Red detected - Sorting\n");
                    if (currentTime - stateStartTime < 140) {
                        Intake2.move_voltage(normalVoltage);
                    } else {
                        Intake2.move_voltage(0);
                    }
                    // Return to idle after 800 ms
                    if (currentTime - stateStartTime >= 800) {
                        stateAuto = IntakeNormal;
                        stateStartTime = currentTime;
                    }
                    break;
                
                case AUTO_SORTING_BLUE:
                    // printf("Blue detected - Sorting\n");
                    // Run full speed for a short burst then stop
                    if (currentTime - stateStartTime < 140) {
                        Intake2.move_voltage(normalVoltage);
                    } else {
                        Intake2.move_voltage(0);
                    }
                    // Return to idle after 800 ms
                    if (currentTime - stateStartTime >= 650) {
                        stateAuto = IntakeNormal;
                        stateStartTime = currentTime;
                    }
                    break;    

                case LB_AUTO:

                if (scoreLB){
                    LBempty = true;
                }

                if (LBempty && readyLB && settled) {
                    ringWaiting = false;
                    
                }

                if (readyLB && settled) {
                    putRingInLB = true;

                }

                if (currentTime - stateStartTime < 240 && !ringWaiting) {
                    Intake1.move_voltage(normalVoltage);
                    Intake2.move_voltage(normalVoltage);
                    ringWaiting = true;

                }
                else {
                    Intake1.move_voltage(12000);
                    Intake2.move_voltage(0);
                }

                if (downLB) {
                    LBempty = true;
                    putRingInLB = false;
                    ringWaiting = false;
                    
                    stateAuto = IntakeNormal;
                    stateStartTime = 0;
                }
                break;
                    


                default:
                    stateAuto = IntakeNormal;
                    stateStartTime = 0;
                    break;
            }



        }
        pros::delay(40);  

    }
        else


    

{
if (!autointake){
        switch (state) {
            case IDLE:
                Intake1.move_voltage(0);
                Intake2.move_voltage(0);

                // Start intake if button is pressed
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                    state = INTAKING;
                } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                    state = REVERSING;
                    stateStartTime = currentTime;
                }

                // Only check for sorting if ring is close
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
                // printf("Red detected - Sorting\n");
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
            // printf("blue detected - Sorting\n");
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

        // printf("State: %d, Hue: %lf, Prox: %d\n", state, hue, proxValue);
        pros::delay(20);
    }
}}