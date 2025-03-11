#include "main.h"
#include "lemlib/api.hpp"
#include "globals.h"
#include "driver.h"
#include "auton.h"

const int numState = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)

int state[numState] = {0, 32, 160};
extern int currStates = 0;
extern int targets = 0;

extern void next(int targetAngle) {
    //add a parameter to this method thats the target angle
	/*currStates += 1;
    if (currStates == numState) {
        currStates = 0;
    }*/
	//next time ur at his house, just delete everything else in this method except for the targets = [] 
	//then just add a parameter to this method thats the target angle
	//and just set targets = (to that parameter you pass in)
    //targets = state[currStates];
	targets = targetAngle;
}

void lift() {
    double kp = 0.8;
    double error = targets - (rotation.get_position()/100.0);
	/*if(targets == states[2]){
		
		top_down_roller.move(0);
	}*/
    double velocity = kp * error;
    lb.move(velocity);
} 

extern void skills(){
	
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	lb.move(127);
	pros::delay(500);
	lb.move(0);

	chassis.moveToPoint(0, -8.35, 500, {.forwards = false, .maxSpeed = 80}, false);
	
	lb.move(-127);
	pros::delay(550);
	lb.move(0); 

	chassis.turnToHeading(88.5, 500);
	chassis.setPose(0, 0, 0);
	pros::delay(300);
	chassis.turnToHeading(0, 100);
	chassis.setPose(0, 0, 0);

	chassis.moveToPoint(0, -20, 850, {.forwards = false, .maxSpeed = 70}, false);
	clamp.set_value(true); //clamps first mogo
	pros::delay(400);

	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(87, 500); //turns to face disk
	afterTurn();
	
	chassis.moveToPoint(0, 20, 1000, {.forwards = true, .maxSpeed = 127}, true);
	intake.move(127);
	rotation.reset_position();
	rotation.set_position(0);

	pros::Task liftControlTask([&]{
        while (true) {
            lift();
            pros::delay(10);
        }
    });

	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(22, 500);
	afterTurn(); //faces second disk
	
	chassis.moveToPose(-3, 52.5, -10, 2300, {.forwards = true, .maxSpeed = 127}, false);//moves to second disk
	chassis.setPose(0, 0, 0);
	pros::delay(800);
	
	chassis.moveToPoint(0, -20.75, 850, {.forwards = false, .maxSpeed = 127}, false);
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(71.5, 500);
	afterTurn();

	
		
	next(32);
	chassis.moveToPoint(-1.25,21, 1800, {.forwards = true, .maxSpeed = 127}, true);
	pros::delay(1400);
	intake.move(0);
	next(165);//deploys arm and scores
	pros::delay(850);
	chassis.setPose(0, 0, 0);
	liftControlTask.suspend();
	chassis.moveToPoint(0, -15, 500, {.forwards = false, .maxSpeed = 127}, false);
	liftControlTask.resume();
	next(0);//resets arm*/
	
	chassis.turnToHeading(88, 500);//turns to face the three rings
	afterTurn();
	intake.move(127);
	pros::delay(500);
	chassis.moveToPoint(0, 70, 2000, {.forwards = true, .maxSpeed = 127}, false);//gets all 3 rings in a row
	/*chassis.setPose(0, 0, 0);

	chassis.turnToHeading(-45, 500);//faces last disk in corner
	afterTurn();

	chassis.moveToPoint(0, 15, 1000, {.forwards = true, .maxSpeed = 127}, false);//gets last disk
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(60, 500);//turns to face corner 
	afterTurn();

	chassis.moveToPoint(0, -10, 1000, {.forwards = false, .maxSpeed = 127}, false);//puts mogo in corner
	chassis.setPose(0, 0, 0);
	piston.set_value(false);//releases mogo

	chassis.moveToPoint(0, 20, 500, {.forwards = true, .maxSpeed = 127}, false);
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(-120, 500);
	afterTurn();

	chassis.moveToPoint(0, 72, 3000, {.forwards = true, .maxSpeed = 127}, false);	//goes to other mogo
	chassis.setPose(0, 0, 0);
	piston.set_value(true);//clamps second mogo

	*/


	

}
void afterTurn(){
	chassis.setPose(0, 0, 0);
	pros::delay(50);
	chassis.turnToHeading(0, 50);
	chassis.setPose(0, 0, 0);
}

void moveArm(int position) {
    // Convert the target position to centidegrees if not already
    int targetPosition = position;
    
    // Use the same PID constants as existing code
    double kp = 0.01;
    bool isThere = false;
    while (!isThere) {
        double error = targetPosition - rotation.get_position();
        double velocity = kp * error;
        
        // Apply the movement
        lb.move(velocity);
        
        // Check if we're close enough to target
        if (std::abs(error) <50) {
            lb.move(0);
            isThere = true;
        }
        
        // Small delay to prevent hogging CPU
        pros::delay(10);
    }
}




extern void redNegative(){

	chassis.moveToPoint(0, 5.7, 200, {.forwards = true, .maxSpeed = 100}, false);
	chassis.setPose(0, 0, 0);

	lb.move(127);
	pros::delay(800);
	lb.move(0);
	pros::delay(50);
	
	chassis.moveToPoint(0, -6.55, 200, {.forwards = false}, false);
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(30 , 400);
	chassis.setPose(0, 0, 0);

	pros::delay(500);
	chassis.setPose(0, 0, 0); 

	lb.move(-127);
	pros::delay(700);
	lb.move(0);
	pros::delay(50);
	
	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(20, 400);
	chassis.setPose(0, 0, 0);
	pros::delay(500);
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, -14.70, 500, {.forwards = false, .maxSpeed = 80}, false);
	chassis.setPose(0, 0, 0);
	pros::delay(500);
	
	chassis.turnToHeading(-30.5, 400);
	chassis.setPose(0, 0, 0);
	pros::delay(500);
	
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, -10, 650, {.forwards = false, .maxSpeed = 50}, false);
	chassis.setPose(0, 0, 0);
	
	clamp.set_value(true);
	
//end of new test

	pros::delay(500);
	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(125, 900, {.maxSpeed = 90}, false);
	pros::delay(600);
	
	intake.move(100);
	chassis.setPose(0, 0, 0);
	pros::delay(300);
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 27, 500, {.forwards = true, .maxSpeed = 80}, false);
	pros::delay(600);
	chassis.setPose(0, 0, 0);
	
	
	chassis.turnToHeading(63, 400);
	chassis.setPose(0, 0, 0);
	pros::delay(800);

	
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 15, 500, {.forwards = true, .maxSpeed = 100}, false);
	pros::delay(600);
	chassis.setPose(0, 0, 0);
	
	
	
	pros::delay(200);
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, -15, 400, {.forwards = false, .maxSpeed = 100}, false);
	pros::delay(200);
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(72, 400);
	chassis.setPose(0, 0, 0);
	pros::delay(200);
	

	chassis.setPose(0, 0, 0);
	intake.move(0);
	pros::delay(200);
	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(0, 400);
	chassis.moveToPose(0, 42, 0, 500, {.forwards = true, .maxSpeed = 90}, false);
	//chassis.moveToPoint(0, 25, 900, {.forwards = true, .maxSpeed = 110}, false);
	
}

extern void blueNegative(){
	chassis.moveToPoint(0, 5.7, 200, {.forwards = true, .maxSpeed = 100}, false);
chassis.setPose(0, 0, 0);

lb.move(127);
pros::delay(800);
lb.move(0);
pros::delay(50);

chassis.moveToPoint(0, -6.55, 200, {.forwards = false}, false);
chassis.setPose(0, 0, 0);

chassis.turnToHeading(-30, 400);
chassis.setPose(0, 0, 0);

pros::delay(500);
chassis.setPose(0, 0, 0);

lb.move(-127);
pros::delay(700);
lb.move(0);
pros::delay(50);

chassis.setPose(0, 0, 0);
chassis.turnToHeading(-20, 400);
chassis.setPose(0, 0, 0);
pros::delay(500);
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, -14.40, 500, {.forwards = false, .maxSpeed = 80}, false);
chassis.setPose(0, 0, 0);
pros::delay(500);

chassis.turnToHeading(31, 400);
chassis.setPose(0, 0, 0);
pros::delay(500);

chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, -11, 650, {.forwards = false, .maxSpeed = 50}, false);
chassis.setPose(0, 0, 0);

clamp.set_value(true);

pros::delay(500);
chassis.setPose(0, 0, 0);
chassis.turnToHeading(-125, 900, {.maxSpeed = 90}, false);
//pros::delay(600);


intake.move(110);
chassis.setPose(0, 0, 0);
pros::delay(300);
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, 30, 500, {.forwards = true, .maxSpeed = 80}, false);
pros::delay(600);
chassis.setPose(0, 0, 0);

chassis.turnToHeading(-66, 400);
chassis.setPose(0, 0, 0);
pros::delay(800);

chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, 11.5, 500, {.forwards = true, .maxSpeed = 100}, false);
pros::delay(1700);
chassis.setPose(0, 0, 0);

chassis.moveToPoint(0, -10, 800, {.forwards = false});
pros::delay(750);
intake.move(0);


chassis.turnToHeading(-58, 700);
chassis.setPose(0, 0, 0);
pros::delay(400);

chassis.setPose(0, 0, 0);
afterTurn();

chassis.moveToPose(0, 45, 0, 1000, {.forwards = true, .maxSpeed = 105}, false);

}


extern void bluePositive(){
	
    //pros::delay(2000);
	chassis.moveToPoint(0, 5.7, 200, {.forwards = true, .maxSpeed = 100}, false);
	chassis.setPose(0, 0, 0);

	lb.move(127);
	pros::delay(800);
	lb.move(0);
	pros::delay(50);
	
	chassis.moveToPoint(0, -6.55, 200, {.forwards = false}, false);
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(30 , 400);
	chassis.setPose(0, 0, 0);

	pros::delay(500);
	chassis.setPose(0, 0, 0); 

	lb.move(-127);
	pros::delay(700);
	lb.move(0);
	pros::delay(50);
}
extern void redPositive(){
    /*chassis.moveToPoint(0, 5.7, 200, {.forwards = true, .maxSpeed = 100}, false);
	chassis.setPose(0, 0, 0);

	arm.move(127);
	pros::delay(800);
	arm.move(0);
	pros::delay(50);
	
	chassis.moveToPoint(0, -6.55, 200, {.forwards = false}, false);
	chassis.setPose(0, 0, 0);

	chassis.turnToHeading(30 , 400);
	chassis.setPose(0, 0, 0);

	pros::delay(500);
	chassis.setPose(0, 0, 0); 

	arm.move(-127);
	pros::delay(700);
	arm.move(0);
	pros::delay(50);

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -10, 400, {.forwards = false, .maxSpeed = 100}, false);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(100, 2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(0, 200);
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0, 30, 400, {.forwards = true, .maxSpeed = 100}, false);

	
    chassis.moveToPoint(0, -6, 400, {.forwards=false, .maxSpeed=100}, false);
    piston.set_value(true);
    //chassis.turnToHeading(45, 4000);
	
*/
pros::delay(3500);
chassis.moveToPoint(0, 5.7, 200, {.forwards = true, .maxSpeed = 100}, false);
chassis.setPose(0, 0, 0);

lb.move(127);
pros::delay(800);
lb.move(0);
pros::delay(50);

chassis.moveToPoint(0, -6.55, 200, {.forwards = false}, false);
chassis.setPose(0, 0, 0);

chassis.turnToHeading(-30, 400);
chassis.setPose(0, 0, 0);

pros::delay(500);
chassis.setPose(0, 0, 0);

lb.move(-127);
pros::delay(700);
lb.move(0);
pros::delay(50);

chassis.setPose(0, 0, 0);
chassis.turnToHeading(-20, 400);
chassis.setPose(0, 0, 0);
pros::delay(500);
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, -14.40, 500, {.forwards = false, .maxSpeed = 80}, false);
chassis.setPose(0, 0, 0);
pros::delay(500);

chassis.turnToHeading(31, 400);
chassis.setPose(0, 0, 0);
pros::delay(500);

chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, -11, 650, {.forwards = false, .maxSpeed = 50}, false);
chassis.setPose(0, 0, 0);

clamp.set_value(true);

pros::delay(500);
chassis.setPose(0, 0, 0);
chassis.turnToHeading(-124, 900, {.maxSpeed = 90}, false);
//pros::delay(600);


intake.move(110);
chassis.setPose(0, 0, 0);
pros::delay(300);
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, 30, 500, {.forwards = true, .maxSpeed = 80}, false);
pros::delay(600);
chassis.setPose(0, 0, 0);

}
