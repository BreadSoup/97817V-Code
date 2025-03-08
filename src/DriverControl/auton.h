#ifndef AUTON_H
#define AUTON_H

#include "definitions.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
bool intake_on = false;   
bool intakerevdone = false;
bool revtop = false;
bool stopbottom = false;
 
inline void intakereverse(void* param) {

   static int state = 1;
   static int stateStartTime = 0;

   const int normalVoltage = 12000;
   const int reverseVoltage = -12000;
   const int velocityThreshold = 10;

   while (true) {
       int currentTime = pros::millis();

		if (!intake_on && !stopbottom) {
           Intake2.move_voltage(0);
           Intake1.move_voltage(12000); }
		else if  (!intake_on && stopbottom) {
		   Intake1.move_voltage(0);
		   Intake2.move_voltage(0);
		  } 

		else if (!intake_on && revtop) {

			Intake1.move_voltage(0);
			Intake2.move_voltage(-12000);
		}
		else {
           switch (state) {
               case 1: 
                   Intake1.move_voltage(normalVoltage);
				   Intake2.move_voltage(normalVoltage);

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
       pros::delay(40);  
   }
}
}



inline void colorsort(void* param){
	double hue = colorsensor.get_hue();
		int colorval = 0;

		bool kickred = true;
		bool kickblue = false;
		while (true){
			double hue = colorsensor.get_hue();


		if (((hue >= 1 && hue < 30) || (hue > 330 && hue <= 360)) && kickred) {
			colorval = 1;  // Red detected.
			pros::delay(300);

			Intake2.move_voltage(-12000);
			pros::delay(100);



		} else if (hue >= 150 && hue <= 260 && kickblue) {
			colorval = 0;  // Blue detected.
		} else {
			colorval = -1; // No clear detection.
		}
		
		// Debug output (optional)
		printf("Hue: %lf, ColorSort: %d\n", hue, colorval);
		
		pros::delay(20);  // Delay to allow other tasks to run.
	}

}

/*
1 = autonsupersafe
2 = skills
3 = autonblueneg
4 = autonredneg
5 = autonbluepos
6 = autonredpos


*/
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




	// chassis.turnToPoint(0, -42.2, globalTimeout, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal

	//chassis.moveToPoint(0, -42.2, globalTimeout,{.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v} ); //drive backwards 
	//chassis.waitUntil(4);
	//state = STATE_HOVER;
	
	//chassis.turnToPoint(0, -153.292, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = maxang_v});// grab score ring on mogo
	// and score held ring on stake
	//chassis.waitUntil(1);
	//state = STATE_UP;

	// chassis.turnToHeading(180, 2000);







	// chassis.waitUntilDone();
	// pros::delay(600);


	//  chassis.moveToPose(0, -58.62, 180, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = 70}); // drive forwards
	

	chassis.moveToPose(0, -47.08, 0, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = 10}); // reverse 
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

	chassis.moveToPoint(-46.865	, 23.934, 4000,{.forwards = clampside,  .maxSpeed = 45, .minSpeed = 0});  //clampmogo
	chassis.waitUntilDone();
	pros::delay(500);

	piston.set_value(clamp); // clamp mogo
	pros::delay(400);

	chassis.turnToPoint(-23.343, 23.547, globalTimeout, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face ring
	intake_on = true;
	chassis.moveToPose(-23.343, 23.547, 78, globalTimeout, {.forwards = intakeside, .lead = -.3, .maxSpeed = 100, .minSpeed = 60}); // score ring/	chassis.waitUntilDone();
	pros::delay(100);
	
	chassis.turnToPoint(-10.035, 45.833, globalTimeout, {.forwards = intakeside}); 

	chassis.moveToPose(-0.035, 45.833, 30, globalTimeout, {.forwards = intakeside, .lead = -.55, .maxSpeed = max_v, .minSpeed = min_v}); // set up to score mogo
	chassis.waitUntil(11);
	chassis.cancelMotion();
	chassis.moveToPoint(25.315, 47.274, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = 10}); //score far ring
	chassis.waitUntilDone();
	pros::delay(400);


	
 
	chassis.turnToPoint(-60.915,  47.08, 1000, {.maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face 3 ringss maybeeeeeeeeee ad back 20 exitrange
	chassis.moveToPose(-60.915, 47.08, 270, 3500, {.maxSpeed = 125}); // score 3 rings change from mt 
	chassis.waitUntil(11);
	chassis.cancelMotion();
	pros::delay(900);	
	
	chassis.moveToPose(-60.915,  47.08, 270, 2500, {.maxSpeed = 75});


	chassis.moveToPoint(-48.915, 47.08, 1500, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 
	chassis.moveToPoint(-61.915, 54.48, 1000, {.maxSpeed = 127}); // score 3 rings change from mt 



	chassis.moveToPose(-43.208, 61.918, 225, 1200, {.forwards = intakeside, .lead = -.3, .maxSpeed = 90, .minSpeed = 10}); // score ring
	
	
	
	chassis.moveToPose(-31.112, 61.658,90, globalTimeout, {.forwards = intakeside, .lead = .3, .maxSpeed = max_v, .minSpeed = min_v}); //set up to put mogo in corner

	chassis.moveToPose( -63.751, 70.658, 90, globalTimeout, {.forwards = clampside, .maxSpeed = 110, .minSpeed = 50}); // score mogo in corner
	chassis.waitUntilDone();
	chassis.turnToHeading(100, 500);
	
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
	chassis.moveToPose(25.326, 24.619, 125,  3000,{.forwards = intakeside,.lead = .4, .maxSpeed = 110, .minSpeed = 50}); //ring half


    intake_on = false;
	chassis.waitUntilDone();
   pros::delay(500);		
  
    chassis.turnToPoint(46.519, -0.256 , 1000);
    chassis.waitUntilDone();

    chassis.moveToPoint(46.519, -0.256, globalTimeout, {.forwards = clampside, .maxSpeed = 60, .minSpeed = 30}); // score mogo in corner
    chassis.waitUntilDone(); 
    pros::delay(300);
    

	piston.set_value(clamp); // clamp mogo
	pros::delay(400);
    intake_on = true;


 

    chassis.turnToPoint(23.498, -23.294, 1500);
     chassis.moveToPose(23.498, -23.294, 90, globalTimeout, {.forwards = intakeside, .lead = -.1, .maxSpeed = 110, .minSpeed = 90}); 
    chassis.waitUntilDone();
    pros::delay(300);

    
    intake_on = false;
    pros::delay(200);
    piston.set_value(unclamp); 

    intake_on = false;


	chassis.moveToPose(67.248, 0.07, 0, 900,{.forwards = intakeside, .lead = -.3, .maxSpeed = 110, .minSpeed = 40}); 

    chassis.moveToPose(67.248, 20.07, 0,  1000,{.forwards = intakeside, .maxSpeed = 110, .minSpeed = 70}); 
	chassis.turnToPoint(67.751, 62.658, 1000);

    chassis.moveToPoint(67.751, 62.658, globalTimeout, {.forwards = intakeside, .maxSpeed = 110, .minSpeed = 100}); // score mogo in corner
	chassis.waitUntil(3);
	chassis.cancelMotion();
	chassis.moveToPose(67.751, 62.658, 30, globalTimeout, {.forwards = intakeside, .lead = .05, .maxSpeed = 110, .minSpeed = 100}); // score mogo in corner

 
	chassis.moveToPose(67.248, 0.07, 0, 3000,{.forwards = clampside, .maxSpeed = 110, .minSpeed = 100}); 
	chassis.turnToPoint(67.248, -20.07, 700 ,{.forwards = clampside, .maxSpeed = 127, .minSpeed = 127});	
	 

	chassis.moveToPose(67.248, -20.07, 0, 3000,{.forwards = clampside, .maxSpeed = 127, .minSpeed = 127}); 

    chassis.moveToPose(67.751, -62.658, 350, globalTimeout, {.forwards = clampside, .maxSpeed = 127, .minSpeed = 127}); // score mogo in corner


 	// chassis.moveToPose(60.751, -62.658, 180, globalTimeout, {.forwards = clampside, .maxSpeed = 127, .minSpeed = 127}); // score mogo in corner
    // chassis.waitUntilDone();
    // pros::delay(500);
    // intake_on = true;
}

inline void stateskills(){
	    
	pros::Task task(intakereverse, (void*)NULL);
	bool intakeside = true;
	bool clampside = false; 
	
	int intakeon = 12000;
	int intakeoff = 0;
	
	bool clamp = true;
	bool unclamp = false;
	
	float max_v = 125;
	float min_v = 30;


	float max_vgoal =40;
	float min_vgoal = 20;

	float max_vsafe = 60;
	float min_vsafe = 30;

	float max_vrisk = 127;
	float min_vrisk = 90;


	
	int maxang_v = 80;
	int minang_v = 1;

	int globalTimeout = 2200;


	// start position
	// chassis.setPose({-59.823, 0, 140})

	// //score alliance stake
	// intake_on = true;
	// pros::delay(650);

	// intake_on = false;


	// chassis.moveToPoint(-46.456, -24.316, globalTimeout, {.forwards = clampside, .maxSpeed = max_v});
	// //TODO ADD CLAMP CODE
	// chassis.waitUntilDone();
	// pros::delay(400);
	// piston.set_value(unclamp); // unclamp mogo
	// intake_on = false;


	
chassis.setPose({-59.823, 0, 90});
//score alliance stake
//lift.move_voltage(intakeon);
intake_on = true;
pros::delay(650);
chassis.moveToPose(-46.865, 0, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v});

chassis.turnToPoint(-46.865, -22.68, globalTimeout,{.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v, .earlyExitRange = 20});
//lift.move_voltage(intakeoff);
intake_on = false;

//move to goal

chassis.moveToPose(-47.865, -25.68, 358, globalTimeout, {.forwards = clampside, .lead =.01,.maxSpeed = max_vgoal, .minSpeed = min_vgoal});
chassis.waitUntilDone();
//  pros::delay(50);
piston.set_value(clamp); // clamp mogo
pros::delay(100);
revtop = true;





	// turn to face ring
	chassis.turnToPoint(-23.956, -23.498, globalTimeout,{.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v, .earlyExitRange = 20});

	//score ring
	intake_on = true;
	revtop = false;

	chassis.moveToPoint(-23.956, -23.498, globalTimeout, {.forwards = intakeside, .maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 1.5});

	chassis.moveToPose(-1.252, -41.293, 120, globalTimeout, {.forwards = intakeside, .lead = .5, .maxSpeed = max_v, .minSpeed = min_v, .earlyExitRange = 12 }); // avoid ladder

	//grab far ring
	chassis.moveToPoint(25.315, -44.274, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v}); // grab ring

	// reverse to mid
	//chassis.moveToPoint(0, -36.5, globalTimeout,{.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v} ); //drive backwards

//	chassis.getPose();
	chassis.moveToPose(0, -39.5, 140, globalTimeout, {.forwards = clampside, .lead =-.1,.maxSpeed = max_v, .minSpeed = min_v}); //drive backwards
	// face wall stake with intake side
	chassis.turnToPoint(0, -153.292, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = maxang_v});// grab score ring on mogo

	// score held ring on stake
	chassis.moveToPose(-0.00, -60.975, 180, globalTimeout, {.forwards = intakeside, .maxSpeed = 90, .minSpeed = 10}); // score ring
	///////////////
	//LB SCORED////
	//////////////

	// todo score again???????

	// reverse to mid to score 3 rings
	
	chassis.moveToPose(0, -47.08, 180, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v}); // reverse 

	// turn to face 3 rings
	chassis.turnToPoint(-58.915, -47.08, 1000, {.maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face 3 ringss maybeeeeeeeeee ad back 20 exitrange

	// score 3 rings
	chassis.moveToPose(-58.915, -47.08, 270, 3500, {.maxSpeed = 125}); // score 3 rings change from mt 

	// cancel to wait at 1st ring
	chassis.waitUntil(11);
	chassis.cancelMotion();
	pros::delay(900);	

	// continue to score 3 rings
	chassis.moveToPose(-58.915,  -47.08, 270, 2500, {.maxSpeed = 75});

	// // reverse to ensure and go back towards 
	// chassis.moveToPoint(-48.915, -47.08, 1500, {.forwards = false , .maxSpeed = 127}); // score 3 rings change from mt 
	// chassis.moveToPoint(-58.915, -47.08, 1000, {.maxSpeed = 127}); // score 3 rings change from mt 


	// get corner ring
	chassis.moveToPose(-25.112, -59.658, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = 50}); //set up to put mogo in corner

	// score mogo in corner
	chassis.moveToPose(-20.112, -59.658, 90, globalTimeout, {.forwards = intakeside, .maxSpeed = 127, .minSpeed = 100});
	chassis.moveToPose(-56.751, -59.658, 90, globalTimeout, {.forwards = clampside, .maxSpeed = 70, .minSpeed = 30}); // score mogo in corner
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(unclamp); // unclamp mogo
	revtop = true;

	//lift.move_voltage(intakeoff);
	intake_on = false;






	//todo PART 2

	// reverse out of corner
	chassis.moveToPoint(-47.474, -58.658, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v}); 

	// turn to face mogo
	chassis.turnToPoint(-47.479, 24.57, globalTimeout, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v}); 	
	revtop = false;


	// clamp mogo 
	// todo check if need vgoal
	chassis.moveToPose(-47.479, 24.57, 180, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v});
	chassis.waitUntilDone();
	pros::delay(600);
	piston.set_value(clamp); // unclamp mogo
	intake_on = false;

	// score ring 
	chassis.turnToPoint(-23.343, 23.547, globalTimeout, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face ring
	intake_on = true;
	chassis.moveToPose(-23.343, 23.547, 78, globalTimeout, {.forwards = intakeside, .lead = -.3, .maxSpeed = 100, .minSpeed = 60}); // score ring

	// face next ring  MAKE FAST
	chassis.turnToPoint(-24.979, 47.888, globalTimeout, {.forwards = intakeside});
	chassis.moveToPoint(-24.979, 47.888,  globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v}); // set up to score mogo

	// turn towards the 2 rings
	chassis.turnToPoint(-59.342, 46.661, globalTimeout, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v}); 

	// score 2 rings
	chassis.moveToPose(-59.342, 46.661, 270, globalTimeout, {.forwards = intakeside, .maxSpeed = 125}); 

	// get side ring
	chassis.moveToPose(-43.208, 61.918, 225, 1200, {.forwards = intakeside, .lead = -.3, .maxSpeed = 90, .minSpeed = 10}); // score ring



	chassis.moveToPose(-20.112,59.658, 100, globalTimeout, {.forwards = intakeside, .maxSpeed = 127, .minSpeed = 100});
	chassis.moveToPose(-66.751, 59.658, 100, globalTimeout, {.forwards = clampside, .maxSpeed = 70, .minSpeed = 30}); // score mogo in corner
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(unclamp); // unclamp mogo
	intake_on = false;



	//todo PART 2.5 get wall stakes









	




}
inline void autoredneg(){
	intake_on = false;

	pros::Task task(intakereverse, (void*)NULL);

	bool intakeside = true;
	bool clampside = false; 
	
	// int intakeon = 12000;
	// int intakeoff = 0;
	
	bool clamp = true;
	bool unclamp = false;
	
	float max_v = 105;
	float max_vgoal = 30;

	float min_v = 60;
	
	int maxang_v = 50;
	int minang_v = 0;
	
	int globalTimeout = 2000;
	intake_on = false;

	
	
	chassis.setPose(54.948, -17.02, 90 );
	
	// Go to mid
	chassis.moveToPoint(40.057, -17.63, 2000, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v});
	
	// Turn to goal
	chassis.turnToPoint(24.5, -24.89, 900, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v});
	
	// Grab goal
	chassis.moveToPoint(24.5, -24.89, 2000, {.forwards = clampside, .maxSpeed = max_vgoal, .minSpeed = min_v});
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(clamp);
	pros::delay(1000);
	pros::delay(900);
	chassis.turnToPoint(23.089,45.271,4000);
	chassis.waitUntilDone();
	pros::delay(400);

	chassis.moveToPoint(23.089,45.271,1000);
	chassis.waitUntil(4);
	intake_on = true;
	chassis.waitUntil(9.5);
	chassis.cancelMotion();
	pros::delay(400);

	chassis.waitUntilDone();
	pros::delay(400);

}

inline void autonbluepos(){
	intake_on = false;

	pros::Task task(intakereverse, (void*)NULL);

	bool intakeside = true;
	bool clampside = false; 
	
	// int intakeon = 12000;
	// int intakeoff = 0;
	
	bool clamp = true;
	bool unclamp = false;
	
	float max_v = 105;
	float max_vgoal = 30;

	float min_v = 60;
	
	int maxang_v = 50;
	int minang_v = 0;
	
	int globalTimeout = 2000;
	intake_on = false;

	
	
	chassis.setPose(54.948, -17.02, 90 );
	
	// Go to mid
	chassis.moveToPoint(40.057, -17.63, 2000, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v});
	
	// Turn to goal
	chassis.turnToPoint(24.5, -24.89, 900, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v});
	
	// Grab goal
	chassis.moveToPoint(24.5, -24.89, 2000, {.forwards = clampside, .maxSpeed = max_vgoal, .minSpeed = min_v});
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(clamp);
	pros::delay(1000);
	intake_on = true;
	pros::delay(900);


	
	// chassis.turnToHeading(240, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
	// chassis.waitUntilDone();


	// piston.set_value(unclamp);
	// intake_on = false;

	
	// // turn and grab ring
	// //chassis.turnToHeading(185, 2000);
	// //pros::delay(1000);

	// chassis.turnToPoint(26.036, -46.227, 2000, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v});
	// chassis.moveToPose(26.036, -46.227, 180, 2000, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v});
	// chassis.waitUntilDone();
	// pros::delay(500);
	
	// // grab mid mog and clamp and score
	// chassis.moveToPoint(23.036, -37.227, 2000, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v});
	
	// chassis.turnToPoint(10, -41.4, 1000, {. forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v});
	// chassis.moveToPoint(10, -42.4, 2000, {.forwards = clampside, .maxSpeed = max_vgoal, .minSpeed = min_v});
	// chassis.waitUntilDone();	
	// pros::delay(500);
	// piston.set_value(clamp);
	// pros::delay(500);

	
	// chassis.moveToPose(24.118, -1.561, 0, 3000, {.forwards = intakeside, .lead = .4, .maxSpeed = max_v, .minSpeed = min_v});
	
	// chassis.waitUntil(3);
	// chassis.cancelMotion();
	 intake_on = false;	
	stopbottom = true;
	chassis.moveToPose(34.918, 11.561, 0, 3000, {.forwards = intakeside, .lead = .3, .maxSpeed = 110, .minSpeed = 110});
	chassis.waitUntil(6);
	intake_on = false;
	piston.set_value(unclamp);

	chassis.turnToPoint(24.5, -24.89, 900, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v});
	
	// Grab goal
	chassis.moveToPoint(24.5, -24.89, 2000, {.forwards = clampside, .maxSpeed = max_vgoal, .minSpeed = min_v});
	chassis.waitUntilDone();
	pros::delay(400);
	piston.set_value(clamp);
	pros::delay(1000);
	intake_on = true;
	pros::delay(900);

	// turn and grab ring
	//chassis.turnToHeading(185, 2000);
	//pros::delay(1000);

	chassis.turnToPoint(25.536, 46.227, 2000, {.forwards = intakeside, .maxSpeed = maxang_v, .minSpeed = minang_v});
	chassis.moveToPoint(25.56, 46.227, 2000, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = min_v});
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.moveToPose(0.118, -1.561, 200, 3000, {.forwards = intakeside, .lead = .2, .maxSpeed = 50, .minSpeed = 10});

	

	









}
inline void autonredpos(){
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




	// chassis.turnToPoint(0, -42.2, globalTimeout, {.forwards = clampside, .maxSpeed = maxang_v, .minSpeed = minang_v}); // turn to face goal

	//chassis.moveToPoint(0, -42.2, globalTimeout,{.forwards = clampside, .maxSpeed = max_v, .minSpeed = min_v} ); //drive backwards 
	//chassis.waitUntil(4);
	//state = STATE_HOVER;
	
	//chassis.turnToPoint(0, -153.292, globalTimeout, {.forwards = intakeside, .maxSpeed = 60, .minSpeed = maxang_v});// grab score ring on mogo
	// and score held ring on stake
	//chassis.waitUntil(1);
	//state = STATE_UP;

	// chassis.turnToHeading(180, 2000);







	// chassis.waitUntilDone();
	// pros::delay(600);


	//  chassis.moveToPose(0, -58.62, 180, globalTimeout, {.forwards = intakeside, .maxSpeed = max_v, .minSpeed = 70}); // drive forwards
	

	chassis.moveToPose(0, -47.08, 0, globalTimeout, {.forwards = clampside, .maxSpeed = max_v, .minSpeed = 10}); // reverse 
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

	
}
static bool doneTime = false;  // This flag indicates that the oscillation period has been measured.

inline void getOscillationTime(void *param) {

	while (true) {
    static bool timerStarted = false;
    static uint32_t startTime = 0;

    double theta = chassis.getPose().theta;

    if (!timerStarted && theta > 90) {
        startTime = pros::millis();
        timerStarted = true;
    }


    if (doneTime && timerStarted) {
        uint32_t oscillationTime = pros::millis() - startTime;
        printf("Oscillation time: %d ms\n", oscillationTime);
        timerStarted = false; 

		break;    
}
	pros::delay(20); }
}

inline void testang(){
	chassis.setPose(0,0,0);
	chassis.turnToPoint(50,0 , 10000, {}, false);
	
	pros::Task atask(getOscillationTime, (void*)NULL);

	while (chassis.isInMotion()) {
        pros::delay(20);
    }

	doneTime = true;

	
}


#endif