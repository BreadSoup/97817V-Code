// main.cpp
#include "main.h"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"

#include "Brain/BrainScreen.h"
#include "DriverControl/definitions.h"
#include "DriverControl/pneumatics.h"
#include "DriverControl/roller.h"
#include "DriverControl/redirect.h"
#include "DriverControl/auton.h"
#include "pros/rtos.h"
#include "pros/screen.hpp"
#include "lemlib/api.hpp" 
#include <cstdlib> // Include the cstdlib header for rand() and srand()
#include <ctime> 

//#include "pros/motor_group.hpp"

//ASSET(bluestep2_txt); 
//ASSET(red2_txt); 
//ASSET(red3_txt); 


//ASSET(bluestep5_txt);
//ASSET(bluestep7_txt); 
//ASSET(drakept1_txt);

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
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
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



//LV_IMG_DECLARE(awesome);
//lv_obj_t* awesome_obj = NULL;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	//master.clear();
	//okJarvisActivateFunnyCatImageOnVexBrain();
	left_mg.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	right_mg.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	lift.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	Redriect.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	encoder.reset(); 
	encoder.reset_position();
	encoder.set_position(1000);

	intializePneumatics();

	//encoder.reset();


	//pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors


  //  pros::lcd::clear();

    // thread to for brain screen and position logging
     pros::Task screenTask([&]() {
      while (true) {
           // print robot location to the brain screen
           pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
           pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
           pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		   pros::lcd::print(3, "horz: %i", horztracking.get_position());
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


 bool intake_on = true;   
 bool intakerevdone = false;

 void intakereverse(void* param) {

    static int state = 1;
    static int stateStartTime = 0;

    const int normalVoltage = 12000;
    const int reverseVoltage = -12000;
    const int velocityThreshold = 10;

    while (true) {
        int currentTime = pros::millis();

        if (!intake_on) {
            lift.move_voltage(0);
        } else {
            switch (state) {
                case 1: 
                    lift.move_voltage(normalVoltage);
                    if (lift.get_actual_velocity() < velocityThreshold) {
                        if (stateStartTime == 0) {
                            stateStartTime = currentTime;
                        }
                        if (stateStartTime != 0 && (currentTime - stateStartTime >= 1000)) {
                            state = 2;
                            stateStartTime = currentTime;
                        }
                    }
                    if (lift.get_actual_velocity() >= velocityThreshold) {
                        stateStartTime = 0;
                    }
                    break;

                case 2:
                    Intake2.move_voltage(reverseVoltage);
                    if (currentTime - stateStartTime >= 1600) {
                        state = 1;
                        stateStartTime = currentTime;
                    }
                    break;

                case 3:  
                    lift.move_voltage(normalVoltage);
                    if (lift.get_actual_velocity() < velocityThreshold) {
                        state = 2; 
                        stateStartTime = currentTime;
                    }
                    if (lift.get_actual_velocity() >= velocityThreshold) {
                        state = 1;  
                        stateStartTime = 0;
                    }
                    break;

                default:
                    state = 1;
                    stateStartTime = 0;
                    break;
            }

			if (intakerevdone) {
				break ;
        }
        pros::delay(20);  
    }
}
 }

void autonomous() {
	//auton();


	pros::Task task(intakereverse, (void*)NULL);
	bool intakeside = true;
	bool clampside = false; 
	
	int intakeon = 12000;
	int intakeoff = 0;
	
	bool clamp = true;
	bool unclamp = false;
	
	float max_v = 105;
	float min_v = 0;
	
	int maxang_v = 70;
	int minang_v = 1;

	int globalTimeout = 2900;



chassis.setPose({-59.823, 0, 90});
	//score alliance stake
	//lift.move_voltage(intakeon);
	intake_on = true;
	pros::delay(650);
	chassis.moveToPose(-46.865, 0, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v});
	
	chassis.turnToPoint(-46.865, -22.68, globalTimeout,{.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v, .earlyExitRange = 20});
	//lift.move_voltage(intakeoff);
	intake_on = false;
	chassis.moveToPose(-46.865, -24.68, 0, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v});
	chassis.waitUntilDone();
	//  pros::delay(50);
	piston.set_value(clamp); // clamp mogo
	//lift.move_voltage(-12000);
	pros::delay(100);
	chassis.turnToPoint(-23.586, -23.74, globalTimeout,{.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal
	//start intake
	//lift.move_voltage(intakeon);
	intake_on = true;

	chassis.moveToPose(-23.586, -23.74, 100, globalTimeout, { .forwards = intakeside, .lead=  -.2, .maxSpeed = 100  , .minSpeed = 60, .earlyExitRange = 1.5}); // score ring
	//pros::delay(380);
	chassis.moveToPose(-0.035, -38.833, 100, globalTimeout, {.forwards = intakeside, .lead = .38, .maxSpeed = 127, .minSpeed = 70}); // set up to score mogo
	chassis.waitUntil(4);
	chassis.cancelMotion();
	/////////////////////////////////////////////
	///LB ON//////////////
	//////////////////////
	
	
	chassis.moveToPoint(23.315, -47.274, globalTimeout, {.forwards = intakeside, .maxSpeed = 110.28, .minSpeed = 2}); // grab ring
	pros::delay(100);
	//lift.move_voltage(127);




	chassis.turnToPoint(0, -43.2, globalTimeout, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal
	chassis.moveToPoint(0, -43.2, globalTimeout,{.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v} ); //drive backwards 
	
	chassis.turnToPoint(0, -53.292, globalTimeout, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = maxang_v});// grab score ring on mogo
	// and score held ring on stake
	
	//pros::delay(300);
	chassis.moveToPose(0, -55.292, 180, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v}); // drive forwards
	pros::delay(500);

	chassis.moveToPose(0, -47.08, 180, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v}); // reverse 
	

	//chassis.turnToHeading(270, globalTimeout, { .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal
	chassis.turnToPoint(-58.915, -47.08, 1000, {.maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face 3 ringss maybeeeeeeeeee ad back 20 exitrange
	chassis.moveToPose(-58.915, -47.08, 270, 3500, {.maxSpeed = 125}); // score 3 rings change from mt 
	chassis.waitUntil(11);
	chassis.cancelMotion();
	pros::delay(900);	
	
	//chassis.moveToPoint(-58.915, -47.08, 2500, {.maxSpeed = 75}); // score 3 rings change from mt 
	chassis.moveToPose(-58.915,  -47.08, 270, 2500, {.maxSpeed = 75});


	// chassis.moveToPoint(-48.915, -47.08, 1000, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 
	// chassis.moveToPoint(-58.915, -47.08, 1000, {.maxSpeed = 127}); // score 3 rings change from mt 


	chassis.moveToPoint(-48.915, -47.08, 1500, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 
	chassis.moveToPoint(-58.915, -47.08, 1000, {.maxSpeed = 127}); // score 3 rings change from mt 



	//chassis.turnToPoint(-47.208,-58.918, 1200, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face ring
	chassis.moveToPose(-43.208, -59.918, 135, 1200, {.forwards = intakeside, .maxSpeed = 90, .minSpeed = 10}); // score ring
	
	
	
	chassis.moveToPose(-31.112, -59.658, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v}); //set up to put mogo in corner

	chassis.moveToPose(-56.751, -59.658, 90, globalTimeout, {.forwards = clampside, .maxSpeed = 110, .minSpeed = 30}); // score mogo in corner
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(unclamp); // unclamp mogo
	//lift.move_voltage(intakeoff);
	intake_on = false;

	
	// chassis.moveToPose(	-46.865	, 0, 0, globalTimeout,{.forwards = intakeside, .lead = -.1, .maxSpeed = 127, .minSpeed = 127}); 
	// chassis.turnToHeading(-270, globalTimeout, { .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal
	// intake_on = false;

	// chassis.moveToPose(-59.23, 0, 90, globalTimeout,{.forwards = intakeside, .lead = -.1, .maxSpeed = max_v, .minSpeed = min_v}); 
	
	


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
	///encoder.set_position(1000);
	intakerevdone = true;
	

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
		RedirectControl();
		/*if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			lift.move(-127);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			lift.move(127);
		}*/
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			lift.move(0);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			Redriect.move(0);
		}
		//pros::screen::print(pros::E_TEXT_MEDIUM, 0, 0, "Encoder Value: %d", encoder.get_value());
		
		//if (master.get_digital(rb)                   // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}