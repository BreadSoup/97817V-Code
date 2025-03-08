// main.cpp
#include "main.h"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"

#include "DriverControl/BrainScreen.h"
#include "DriverControl/definitions.h"
#include "DriverControl/pneumatics.h"
#include "DriverControl/roller.h"
#include "DriverControl/redirect.h"
#include "DriverControl/auton.h"
#include "DriverControl/LadyBrown.h"
#include "pros/rtos.h"
#include "pros/screen.hpp"
#include "lemlib/api.hpp" 
#include <cstdlib> 
#include <ctime> 
#include "main.h"
 #include <stdio.h>


 #include <vector>
 #include <string>

void initialize() {
	//pros::lcd::initialize();
	
    
    // Autonomous can perform additional actions here
    // For example, delay to let the task run for a set period
   
 	//okJarvisActivateFunnyCatImageOnVexBrain();
	left_mg.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	right_mg.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	lift.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	Redriect.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	encoder.reset(); 
	encoder.reset_position();
	encoder.set_position(1000);
	pros::lcd::clear();


	intializePneumatics();
	lv_init();
	setup_lcd();
	pros::Task atask(colorsort, (void*)NULL);


    // Setup our custom interface.

	//encoder.reset();


	//pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors
	
 


    pros::lcd::clear();
  //pros::Task autoTask(autoSelectorTask, (void*)NULL);

    // thread to for brain screen and position logging
     pros::Task screenTask([&]() {
      while (true) {
			
           // print robot location to the brain screen
     chassis.getPose().x; // x
          chassis.getPose().y; // y
          chassis.getPose().theta; // heading
		  lv_task_handler(); // Process LVGL tasks
		  update_motor_data();


		///   pros::lcd::printf(3, "horz: %i", horztracking.get_position());
		   // log position telemetry
           lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
           // delay to save resources
           pros::delay(20);

      }});
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
    std::string selectedAutoStr = "Selected auto: " + getAutoName(cycle_number);
    // Update the LVGL auto label with the selected autonomous mode.
    lv_label_set_text(auto_label, selectedAutoStr.c_str());
  skill2();
  //autonbluepos();
  //autoredneg();
  //autonredpos();


    // // Run the corresponding autonomous routine.
    // switch (cycle_number) {
    //     case 0:
    //         autonsupersafe();
    //         break;
    //     case 1:
    //         skills();
    //         break;
    //     case 3:
    //         autonblueneg();
    //         break;
    //     case 4:
    //         autonredneg();
    //         break;
    //     case 5:
    //         autonbluepos();
    //         break;
    //     case 6:
    //         autonredpos();
    //         break;
    //     default:
    //         // If the cycle number is invalid, you can choose a default auto or do nothing.
    //         pros::lcd::print(2, "Invalid auto selection!");
    //         break;

 
    // }

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	bool pistonState;
 	bool reversedSteering;
	intakerevdone = true;
    
 
	lv_task_handler();

	

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
 		// Arcade; control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			reversedSteering = false;
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			reversedSteering = true;
		}
		//if (!reversedSteering){
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);  
		//}
		/*else {
			left_mg.move((dir*-1) + turn);                      // Sets left motor voltage
			right_mg.move((dir*-1) - turn); 
		}*/
		/*
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			pistonState = !pistonState; // Toggle the state of the piston
			piston.set_value(pistonState); // Activate or deactivate the solenoid
		}
		*/
		clampSolenoid();
		rollerControl();
		updateStateFromInput();
		RedirectControl();
	//	RedirectControl();

	 
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			lift.move(0);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			Redriect.move(0);
		}
 
		pros::delay(20);                               // Run for 20 ms then update
	}
}