#include "main.h"
#include "pros/rtos.h"
#include "definitions.h"

// Motor ports vector.
std::vector<int> motorPorts = { -1, -21, 11, 9, -18, -17, 15, 19, -13 };

// LVGL objects for motor labels, battery display, and team image.
std::vector<lv_obj_t*> motorLabels;  
lv_obj_t* batteryLabel = nullptr;
lv_obj_t* teamImageObj = nullptr;

// ---------------------- Global UI Elements ----------------------
// Global cycle number and its labels.
static int cycle_number = 0;
static lv_obj_t* cycle_label = nullptr;
static lv_obj_t* auto_label = nullptr;  // Displays the current autonomous mode

// ---------------------- Helper Functions ----------------------

// Maps cycle number to autonomous name.
std::string getAutoName(int cycle) {
    switch(cycle) {
        case 1: return "autonsupersafe";
        case 2: return "skills";
        case 3: return "autonblueneg";
        case 4: return "autonredneg";
        case 5: return "autonbluepos";
        case 6: return "autonredpos";
        default: return "Invalid";
    }
}

// Updates the auto_label text based on the current cycle_number.
void update_auto_label() {
    std::string autoStr = "Auto: " + getAutoName(cycle_number);
    lv_label_set_text(auto_label, autoStr.c_str());
}

// ---------------------- LVGL Callback Functions ----------------------

// Callback for background color animation.
static void bg_anim_cb(void* var, int32_t v) {
    lv_obj_t* obj = static_cast<lv_obj_t*>(var);
    lv_color_t color = lv_color_make(v, v / 2, 0);  // Orange color: (v, v/2, 0)
    lv_obj_set_style_bg_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

// Callback to set opacity.
static void set_opa_cb(void* var, int32_t v) {
    lv_obj_set_style_opa(static_cast<lv_obj_t*>(var), v, LV_PART_MAIN | LV_STATE_DEFAULT);
}

// ---------------------- Button Event Callbacks ----------------------
static void btn_plus_event_cb(lv_event_t* e) {
    cycle_number++;
    char buf[16];
    sprintf(buf, "%d", cycle_number);
    lv_label_set_text(cycle_label, buf);
    update_auto_label();  // Update auto label when cycle number changes.
}

static void btn_minus_event_cb(lv_event_t* e) {
    cycle_number--;
    char buf[16];
    sprintf(buf, "%d", cycle_number);
    lv_label_set_text(cycle_label, buf);
    update_auto_label();  // Update auto label when cycle number changes.
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
    motorLabels.resize(motorPorts.size());
    static lv_style_t motorLabelStyle;
    lv_style_init(&motorLabelStyle);
    lv_style_set_text_color(&motorLabelStyle, lv_color_white());
    lv_style_set_text_font(&motorLabelStyle, &lv_font_montserrat_12);
    lv_style_set_radius(&motorLabelStyle, 8);  // Rounded corners

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
    // Uncomment and set the source if you have a team logo asset.
    // lv_img_set_src(teamImageObj, &Team);
    lv_obj_align(teamImageObj, LV_ALIGN_BOTTOM_RIGHT, 0, -10);

    // ---------------------- Cycle Number and Button Setup ----------------------
    // Create the auto label positioned above the cycle number.
    auto_label = lv_label_create(scr);
    lv_label_set_text(auto_label, "Auto: Invalid");
    lv_obj_align(auto_label, LV_ALIGN_BOTTOM_RIGHT, -40, -100);  // Adjust offsets as needed

    // Create the cycle number label positioned below the auto label.
    cycle_label = lv_label_create(scr);
    lv_label_set_text(cycle_label, "0");
    lv_obj_align(cycle_label, LV_ALIGN_BOTTOM_RIGHT, -40, -80);  // Adjust offsets as needed

    // Create the plus button.
    lv_obj_t* btn_plus = lv_btn_create(scr);
    lv_obj_set_size(btn_plus, 30, 30);  // Tiny button size.
    lv_obj_align(btn_plus, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_plus, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_plus, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_plus, btn_plus_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_plus = lv_label_create(btn_plus);
    lv_label_set_text(label_plus, "+");
    lv_obj_center(label_plus);

    // Create the minus button.
    lv_obj_t* btn_minus = lv_btn_create(scr);
    lv_obj_set_size(btn_minus, 30, 30);
    lv_obj_align(btn_minus, LV_ALIGN_BOTTOM_RIGHT, -50, -10);  // Positioned to the left of plus.
    lv_obj_set_style_bg_color(btn_minus, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_minus, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_minus, btn_minus_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_minus = lv_label_create(btn_minus);
    lv_label_set_text(label_minus, "-");
    lv_obj_center(label_minus);
}
 
// ---------------------- Motors ----------------------
// Create a vector of motors using the ports from motorPorts.
std::vector<pros::Motor> motors = {
    pros::Motor(motorPorts[0], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[1], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[2], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[3], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[4], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[5], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[6], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[7], pros::v5::MotorGears::blue),
    pros::Motor(motorPorts[8], pros::v5::MotorGears::blue)
};

// ---------------------- Motor Data Update ----------------------
void update_motor_data() {
    // Loop over each motor.
    for (size_t i = 0; i < motorPorts.size(); ++i) {
        double temp = motors[i].get_temperature();         // Temperature in °C.
        double rpm = motors[i].get_actual_velocity();        // RPM.
        double current_draw = motors[i].get_current_draw();    // Current draw in Amps.
        double relative_pos = motors[i].get_position();        // Relative position.

        // Check if any data is invalid (e.g., motor unplugged).
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
        lv_label_set_text(motorLabels[i], display_str.c_str());
    }

    // Update battery percentage.
    int battery_percent = pros::battery::get_capacity(); 
    std::string battery_str = "Battery: " + std::to_string(battery_percent) + "%";
    lv_label_set_text(batteryLabel, battery_str.c_str());

    // Process LVGL tasks.
    lv_task_handler();
}
