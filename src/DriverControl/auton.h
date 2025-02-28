#ifndef AUTON_H
#define AUTON_H

#include "definitions.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
bool intake_on = true;   
bool intakerevdone = false;
 
inline void intakereverse(void* param) {

   static int state = 1;
   static int stateStartTime = 0;

   const int normalVoltage = 12000;
   const int reverseVoltage = -12000;
   const int velocityThreshold = 10;

   while (true) {
       int currentTime = pros::millis();

       if (!intake_on) {
           Intake2.move_voltage(0);
           Intake1.move_voltage(12000);
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


inline void autonsupersafe(){
    piston.set_value(false);
    drivetrain.move(-50);
    pros::c::delay(950);
    drivetrain.move(0);
    //piston2.set_value(false);
    pros::c::delay(200);
    piston.set_value(true);
    pros::c::delay(1500);
    lift.move(127);
    pros::c::delay(1000);
    Redriect.move_voltage(-100);
    right_mg.move_voltage(12000);
    left_mg.move_voltage(-12000);
    pros::delay(500);
    right_mg.move_voltage(0);
    left_mg.move_voltage(0);
    drivetrain.move(40);
    //pros:pros::c::delay(1000);
    Redriect.move_voltage(0);
    //right_mg.move_voltage(127);
    pros::delay(720);
    drivetrain.move_voltage(0);
    lift.move_voltage(30);
}

inline void skills(){
    
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

	chassis.moveToPose(-23.586, -23.74, 100, 1000, { .forwards = intakeside, .lead=  -.2, .maxSpeed = 100  , .minSpeed = 60, .earlyExitRange = 1.5}); // score ring
	//pros::delay(380);
	chassis.moveToPose(-0.035, -38.833, 100, globalTimeout, {.forwards = intakeside, .lead = .38, .maxSpeed = max_v, .minSpeed = min_v}); // set up to score mogo
	chassis.waitUntil(4);
	chassis.cancelMotion();
	/////////////////////////////////////////////
	///LB ON//////////////
	//////////////////////
//	preventReverse = true;
	//state = STATE_MID;



	
	
	chassis.moveToPoint(25.315, -44.274, globalTimeout, {.forwards = intakeside, .maxSpeed = 100.28, .minSpeed = 2}); // grab ring
	pros::delay(100);
	//lift.move_voltage(127);




	chassis.turnToPoint(0, -42.2, globalTimeout, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal

	chassis.moveToPoint(0, -42.2, globalTimeout,{.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v} ); //drive backwards 
	//chassis.waitUntil(4);
	//state = STATE_HOVER;
	
	//chassis.turnToPoint(0, -153.292, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = maxang_v});// grab score ring on mogo
	// and score held ring on stake
	//chassis.waitUntil(1);
	//state = STATE_UP;

	chassis.turnToHeading(180, 2000);







	chassis.waitUntilDone();
	pros::delay(600);


	 chassis.moveToPose(0, -55.292, 180, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v}); // drive forwards
	

	chassis.moveToPose(0, -47.08, 180, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = 10}); // reverse 
	//state = STATE_DOWN;

	chassis.turnToPoint(-58.915, -47.08, 1000, {.maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face 3 ringss maybeeeeeeeeee ad back 20 exitrange
	chassis.moveToPose(-58.915, -47.08, 270, 3500, {.maxSpeed = 125}); // score 3 rings change from mt 
	chassis.waitUntil(11);
	chassis.cancelMotion();
	pros::delay(900);	
	
	//preventReverse = false;
	
	chassis.moveToPose(-58.915,  -47.08, 270, 2500, {.maxSpeed = 75});


	

	chassis.moveToPoint(-48.915, -47.08, 1500, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 
	chassis.moveToPoint(-58.915, -47.08, 1000, {.maxSpeed = 127}); // score 3 rings change from mt 
	//chassis.moveToPoint(-20.915, -47.08, 1500, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 



	//chassis.moveToPose(-43.208, -59.918, 135, 1200, {.forwards = intakeside, .maxSpeed = 90, .minSpeed = 10}); // score ring
	
	
	
	chassis.moveToPose(-25.112, -59.658, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = 50}); //set up to put mogo in corner
	chassis.moveToPose(-20.112, -59.658, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = 127, .minSpeed = 100});
	chassis.moveToPose(-56.751, -59.658, 90, globalTimeout, {.forwards = clampside, .maxSpeed = 70, .minSpeed = 30}); // score mogo in corner
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(unclamp); // unclamp mogo
	//lift.move_voltage(intakeoff);
	intake_on = false;

	


// todo finish 2nd half 

	chassis.moveToPose(	-46.865	, 0, 0, globalTimeout,{.forwards = intakeside, .lead = -.1, .maxSpeed = 105, .minSpeed = 0});  //clampmogo
	pros::delay(600);
	chassis.turnToPoint(-46.865, 25.934, globalTimeout, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face ring
	
	
	
	
	
	chassis.waitUntilDone();

	chassis.moveToPoint(-46.865	, 22.934, 4000,{.forwards = clampside,  .maxSpeed = 45, .minSpeed = 0});  //clampmogo
	chassis.waitUntilDone();
	pros::delay(500);

	piston.set_value(clamp); // clamp mogo
	pros::delay(400);

	chassis.turnToPoint(-23.343, 23.547, globalTimeout, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face ring
	intake_on = true;
	chassis.moveToPose(-23.343, 23.547, 78, globalTimeout, {.forwards = intakeside, .lead = -.3, .maxSpeed = 100, .minSpeed = 60}); // score ring
//	chassis.waitUntilDone();
	//pros::delay(500);
	

	chassis.moveToPose(-0.035, 38.833, 75, globalTimeout, {.forwards = intakeside, .lead = -.05, .maxSpeed = max_v, .minSpeed = min_v}); // set up to score mogo
	chassis.waitUntil(4);
	chassis.cancelMotion();
	chassis.moveToPoint(25.315, 44.274, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = 10}); //score far ring
	chassis.waitUntilDone();
	pros::delay(400);


	
 
	chassis.turnToPoint(-58.915,  47.08, 1000, {.maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face 3 ringss maybeeeeeeeeee ad back 20 exitrange
	chassis.moveToPose(-58.915, 47.08, 270, 3500, {.maxSpeed = 125}); // score 3 rings change from mt 
	chassis.waitUntil(11);
	chassis.cancelMotion();
	pros::delay(900);	
	
	chassis.moveToPose(-58.915,  47.08, 270, 2500, {.maxSpeed = 75});


	chassis.moveToPoint(-48.915, 47.08, 1500, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 
	chassis.moveToPoint(-58.915, 47.08, 1000, {.maxSpeed = 127}); // score 3 rings change from mt 



	chassis.moveToPose(-43.208, 61.918, 225, 1200, {.forwards = intakeside, .lead = -.3, .maxSpeed = 90, .minSpeed = 10}); // score ring
	
	
	
	chassis.moveToPose(-31.112, 61.658, 90, globalTimeout, {.forwards = intakeside, .lead = .3, .maxSpeed = max_v, .minSpeed = min_v}); //set up to put mogo in corner

	chassis.moveToPose( -60.751, 62.658, 90, globalTimeout, {.forwards = clampside, .maxSpeed = 110, .minSpeed = 30}); // score mogo in corner
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(unclamp); // unclamp mogo
	//lift.move_voltage(intakeoff);
	intake_on = false;
    Intake2.move_voltage(-12000);
    chassis.waitUntil(6);
    Intake2.move_voltage(0);
     //   Intake2.move_voltage(-12000);

    
    






    //todo 3rd section 
	chassis.moveToPoint(3.085, 45.658, globalTimeout, {.forwards = intakeside, .maxSpeed = 110, .minSpeed = 70}); // score mogo in corner
    chassis.waitUntilDone();
	pros::delay(300);
//chassis.turnToHeading(0 , 4000);

    chassis.turnToPoint(24.326, 22.619, 1000);
	chassis.moveToPose(24.326, 22.619, 125,  3000,{.forwards = intakeside, .maxSpeed = 110, .minSpeed = 50}); //ring half


    intake_on = false;

  //  pros::delay(500);
  
    chassis.turnToPoint(46.519, -0.256 , 2000);
    chassis.waitUntilDone();

    chassis.moveToPoint(46.519, -0.256, globalTimeout, {.forwards = clampside, .maxSpeed = 60, .minSpeed = 30}); // score mogo in corner
    chassis.waitUntilDone(); 
    pros::delay(300);
    

	piston.set_value(clamp); // clamp mogo
	pros::delay(400);
    intake_on = true;


 

    chassis.turnToPoint(23.498, -23.294, 1500);
    chassis.waitUntilDone();
    chassis.moveToPose(23.498, -23.294, 90, globalTimeout, {.forwards = intakeside, .lead = -.1, .maxSpeed = 110, .minSpeed = 90}); 
    chassis.waitUntilDone();
    pros::delay(300);

    
    intake_on = false;
    pros::delay(200);
    piston.set_value(unclamp); 

    intake_on = false;


    chassis.moveToPose(57.248, 20.07, 0,  3000,{.forwards = intakeside, .maxSpeed = 110, .minSpeed = 50}); 

    chassis.moveToPose(60.751, 62.658, 0, globalTimeout, {.forwards = intakeside, .maxSpeed = 110, .minSpeed = 100}); // score mogo in corner
	chassis.waitUntilDone();
    pros::delay(300);
	piston.set_value(unclamp); 

	chassis.moveToPose(57.248, 20.07, 0,                              3000,{.forwards = clampside, .maxSpeed = 110, .minSpeed = 100}); 

    chassis.moveToPose(60.751, 62.658, 0, globalTimeout, {.forwards = clampside, .maxSpeed = 110, .minSpeed = 100}); // score mogo in corner


 	// chassis.moveToPose(60.751, -62.658, 180, globalTimeout, {.forwards = clampside, .maxSpeed = 127, .minSpeed = 127}); // score mogo in corner
    // chassis.waitUntilDone();
    // pros::delay(500);
    // intake_on = true;
}

#endif