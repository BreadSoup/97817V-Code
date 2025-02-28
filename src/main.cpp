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

 // todo fix ports
 std::vector<int> motorPorts = { -1, -2, -3, 4, 5, 6, -10, -20 };

std::vector<lv_obj_t*> motorLabels;  
lv_obj_t* batteryLabel = nullptr;
lv_obj_t* teamImageObj = nullptr;

// ---------------------- New Global UI Elements ----------------------
// Global cycle number and its label
static int cycle_number = 0;
static lv_obj_t* cycle_label = nullptr;

// ---------------------- LVGL Callback Functions ----------------------
static void bg_anim_cb(void* var, int32_t v) {
    lv_obj_t* obj = static_cast<lv_obj_t*>(var);
    // Orange color animation: (v, v/2, 0)
    lv_color_t color = lv_color_make(v, v / 2, 0);
    lv_obj_set_style_bg_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void set_opa_cb(void* var, int32_t v) {
    lv_obj_set_style_opa(static_cast<lv_obj_t*>(var), v, LV_PART_MAIN | LV_STATE_DEFAULT);
}

// ---------------------- Button Event Callbacks ----------------------
static void btn_plus_event_cb(lv_event_t* e) {
    cycle_number++;
    char buf[16];
    sprintf(buf, "%d", cycle_number);
    lv_label_set_text(cycle_label, buf);
}

static void btn_minus_event_cb(lv_event_t* e) {
    cycle_number--;
    char buf[16];
    sprintf(buf, "%d", cycle_number);
    lv_label_set_text(cycle_label, buf);
}

// ---------------------- LVGL Setup Function ----------------------
void setup_lcd() {
    srand(time(NULL));
    lv_obj_t* scr = lv_scr_act();

    // ---------------------- Background Setup ----------------------
    static lv_style_t bgStyle;
    lv_style_init(&bgStyle);
    lv_style_set_bg_color(&bgStyle, lv_color_make(0xFF, 0x66, 0x00));           // VLC Orange
    lv_style_set_bg_grad_color(&bgStyle, lv_color_make(0xFF, 0x66, 0x00));      // VLC Orange gradient
    lv_style_set_bg_grad_dir(&bgStyle, LV_GRAD_DIR_VER);                       // Vertical gradient
    lv_obj_add_style(scr, &bgStyle, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ---------------------- Background Animation ----------------------
    static lv_anim_t bgAnim;
    lv_anim_init(&bgAnim);
    lv_anim_set_var(&bgAnim, scr);
    lv_anim_set_values(&bgAnim, 0, 255);        // Animate orange value from 0 to 255
    lv_anim_set_time(&bgAnim, 10000);           // Duration of 10 seconds
    lv_anim_set_playback_time(&bgAnim, 10000);  // Same for playback
    lv_anim_set_repeat_count(&bgAnim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&bgAnim, bg_anim_cb);
    lv_anim_set_path_cb(&bgAnim, lv_anim_path_linear);
    lv_anim_start(&bgAnim);

    // ---------------------- Motor Labels Setup ----------------------
    // Resize our vector to match the number of motor ports.
    motorLabels.resize(motorPorts.size());
    
    static lv_style_t motorLabelStyle;
    lv_style_init(&motorLabelStyle);
    lv_style_set_text_color(&motorLabelStyle, lv_color_white());
    lv_style_set_text_font(&motorLabelStyle, &lv_font_montserrat_12);
    lv_style_set_radius(&motorLabelStyle, 8);  // Rounded corners

    // Create a label for each motor based on the ports in our vector.
    for (size_t i = 0; i < motorPorts.size(); ++i) {
        motorLabels[i] = lv_label_create(scr);
        std::string initialText = "Motor " + std::to_string(motorPorts[i]) + ": Initializing...";
        lv_label_set_text(motorLabels[i], initialText.c_str());
        lv_obj_add_style(motorLabels[i], &motorLabelStyle, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_pos(motorLabels[i], 10, 40 + (i * 25)); // Position each label vertically

        // Fade-in animation for the label:
        static lv_anim_t motorFadeAnim;  // Using a single anim instance for simplicity
        lv_anim_init(&motorFadeAnim);
        lv_anim_set_var(&motorFadeAnim, motorLabels[i]);
        lv_anim_set_values(&motorFadeAnim, 0, LV_OPA_COVER);
        lv_anim_set_time(&motorFadeAnim, 1000);
        lv_anim_set_exec_cb(&motorFadeAnim, set_opa_cb);
        lv_anim_set_path_cb(&motorFadeAnim, lv_anim_path_ease_in);
        lv_anim_start(&motorFadeAnim);
    }
 
    // ---------------------- Battery Label Setup ----------------------
    static lv_style_t batteryStyle;
    lv_style_init(&batteryStyle);
    lv_style_set_text_color(&batteryStyle, lv_color_white());
    lv_style_set_text_font(&batteryStyle, &lv_font_montserrat_20);
    lv_style_set_radius(&batteryStyle, 10);
    lv_style_set_bg_color(&batteryStyle, lv_color_make(0x00, 0xFF, 0x00));        
    lv_style_set_bg_grad_color(&batteryStyle, lv_color_make(0x00, 0x80, 0x00));    
    lv_style_set_bg_grad_dir(&batteryStyle, LV_GRAD_DIR_VER);
 
    batteryLabel = lv_label_create(scr);
    lv_label_set_text(batteryLabel, "Battery: ");
    lv_obj_add_style(batteryLabel, &batteryStyle, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_pos(batteryLabel, 190, 10);
 
    // ---------------------- Team Name Image Setup ----------------------
    teamImageObj = lv_img_create(scr);
    // If you have an image asset for the team logo, uncomment the following line:
    // lv_img_set_src(teamImageObj, &Team);
    lv_obj_align(teamImageObj, LV_ALIGN_BOTTOM_RIGHT, 0, -10);

    // ---------------------- Cycle Number and Button Setup ----------------------
    // Create the cycle number label positioned above the buttons
    cycle_label = lv_label_create(scr);
    lv_label_set_text(cycle_label, "0");
    lv_obj_align(cycle_label, LV_ALIGN_BOTTOM_RIGHT, -40, -80);  // Adjust offsets as needed

    // Create the plus button
    lv_obj_t* btn_plus = lv_btn_create(scr);
    lv_obj_set_size(btn_plus, 30, 30);  // Tiny button size
    lv_obj_align(btn_plus, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_plus, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_plus, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_plus, btn_plus_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_plus = lv_label_create(btn_plus);
    lv_label_set_text(label_plus, "+");
    lv_obj_center(label_plus);

    // Create the minus button
    lv_obj_t* btn_minus = lv_btn_create(scr);
    lv_obj_set_size(btn_minus, 30, 30);
    lv_obj_align(btn_minus, LV_ALIGN_BOTTOM_RIGHT, -50, -10);  // Positioned to the left of plus
    lv_obj_set_style_bg_color(btn_minus, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_minus, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_minus, btn_minus_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_minus = lv_label_create(btn_minus);
    lv_label_set_text(label_minus, "-");
    lv_obj_center(label_minus);
}
 
std::vector<pros::Motor> motors = {
    pros::Motor(motorPorts[0], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[1], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[2], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[3], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[4], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[5], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[6], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[7], pros::v5::MotorGears::blue)
};

void update_motor_data() {
    // Loop over the vector of motor ports (which defines the number of motors)
    for (size_t i = 0; i < motorPorts.size(); ++i) {
        double temp = motors[i].get_temperature();         // Temperature in °C
        double rpm = motors[i].get_actual_velocity();        // RPM
        double current_draw = motors[i].get_current_draw();    // Current draw in Amps
        double relative_pos = motors[i].get_position();        // Relative position

        // Check if any data is invalid (e.g., if the motor is unplugged)
        bool unplugged = std::isinf(temp) || std::isinf(rpm) || std::isinf(current_draw) || std::isinf(relative_pos);

        std::string display_str;
        if (unplugged) {
            display_str = "Motor " + std::to_string(motorPorts[i]) + ": Unplugged";
        } else {
            display_str = "Motor " + std::to_string(motorPorts[i]) + ": ";
            display_str += std::to_string(temp) + "°C | ";
            display_str += std::to_string(static_cast<int>(rpm)) + " RPM | ";
            display_str += std::to_string(current_draw) + "A | ";
            display_str += std::to_string(static_cast<int>(relative_pos));
        }

        // Update the corresponding LVGL label from our vector
        lv_label_set_text(motorLabels[i], display_str.c_str());
    }

    // Update battery percentage using the global battery label
    int battery_percent = pros::battery::get_capacity(); 
    std::string battery_str = "Battery: " + std::to_string(battery_percent) + "%";
    lv_label_set_text(batteryLabel, battery_str.c_str());

    // Process LVGL tasks
    lv_task_handler();
}
 
void initialize() {
	//pros::lcd::initialize();
	
    
    // Autonomous can perform additional actions here
    // For example, delay to let the task run for a set period
   
	//master.clear();
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
	//autonsupersafe();

	skills();

 


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
    
	//pros::Task redirectTask(RedirectControl, (void*)NULL);

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
		//pros::screen::print(pros::E_TEXT_MEDIUM, 0, 0, "Encoder Value: %d", encoder.get_value());
		
		//if (master.get_digital(rb)                   // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}