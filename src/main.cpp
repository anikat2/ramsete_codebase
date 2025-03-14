#include "main.h"


// ================================= Views ================================= //


// ========================= Competition Functions ========================= //

void initialize() {	

	pros::delay(100);
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	rotation.reset_position();
	rotation.set_position(0);
	controller.clear();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	
	/**/
	pros::Task screenTask([&]() {
    while (true) {
        double x_pos = chassis.getPose().x;
		double y_pos = chassis.getPose().y;

		// Print X, Y, and arm position
		pros::lcd::print(0, "X: %.2f", x_pos);
		pros::lcd::print(1, "Y: %.2f", y_pos);
		pros::lcd::print(2, "Rotation Pos: %i", rotation.get_position());
		controller.print(2, 0, "Heading: %.2f", imu.get_heading());
    	pros::delay(1000);
    }
	});

}

void disabled() {}

void competition_initialize() {
	// Focus auton selector on screen
	//selector.focus();
	//autonSelector();
}

void autonomous() {
	// Run the selected autonomous function
	//selector.run_auton();
	/*
	int selectedAuton = 1;
	
	if(selectedAuton == 1){
		redPositive();
	}
	else if(selectedAuton == 2){
		redNegative();
	}
	else if(selectedAuton == 3){
		bluePositive();
	}
	else if(selectedAuton == 4){
		blueNegative();
	}
	else{
		skills();
	}
		*/
	//divide desired angle by 2
	//multiply desired x by 2
	/*
	RamseteController ramsete(2, 0.4);
	lemlib::Pose endPose = {48,24,45};
	chassis.setPose(0,0,0);
	ramsete.moveToPose(endPose);
	pros::delay(1000);	
	chassis.setPose(0,0,0);
	*/
	lb.move(127);
	pros::delay(500);
	lb.move(0);
	pros::delay(100);
	lb.move(-127);
	pros::delay(500);
	lb.move(0);
	//chassis.turnToHeading(-275, 3000);
	chassis.moveToPoint(0, -8, 500, {.forwards = false, .maxSpeed = 50}, false);
	chassis.turnToHeading(88.5, 500);
	chassis.setPose(0,0,0);
	chassis.moveToPoint(-24, 0, 1200, {.forwards = false, .maxSpeed = 50}, false);
	clamp.set_value(true);

	pros::delay(1000);
	clamp.set_value(false);


}
const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int auton = 177;
int zero = 0;
int states[numStates] = {0, 32, 160};
extern int currState = 0;
extern int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) 
        currState = 0;
    target = states[currState];
}

void liftControl() {
    double kp = 0.8;
    double error = target - (rotation.get_position()/100.0);
	if(target == states[2])
		intake.move(0);
    double velocity = kp * error;
    lb.move(velocity);
} 
void nextforcustom(int targetAngle) {
	target = targetAngle;
}
void opcontrol() {
	/** 
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	controller.clear();
	rotation.reset_position();
	rotation.set_position(0);
	
	pros::Task liftControlTask([&]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
		
    });
	
	pros::Task controllerTask([&]{
		while (true) {
			lemlib::Pose pose = chassis.getPose();
			//controller.print(0 ,0, "Angle: %f",  pose.theta);
			controller.print(1, 0, "Y: %f", pose.y);
			controller.print(2, 0, "X: %f", pose.x);
			controller.print(0, 0, "Rotation: %d", rotation.get_position());
		}
	});
	pros::Task custom([&]{
		while (true) {
			liftControl();
			pros::delay(10);
		}
	});
	custom.suspend();
	while (true) {
		pros::lcd::print(0, "OpControl Running");
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			liftControlTask.resume();
			custom.suspend();
			nextState();
		}
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			liftControlTask.suspend();
			custom.resume();
			intake.move(-5);
			pros::delay(200);
			intake.move(0);
			nextforcustom(210);

		}
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			liftControlTask.suspend();
			custom.resume();
			intake.move(-5);
			pros::delay(200);
			intake.move(0);
			nextforcustom(48);
		}
		//controller.print(1, 0, "Arm Pos: %.2f", arm.get_position());
		//controller.print(0, 0, "Rotation: %04d", (int)rotation.get_position());
		
		drive();
		roller();
		togglePiston();

	}
	*/
	//chassis.turnToHeading(90, 400);
	autonomous();
	//drive();
	//chassis.moveToPoint(0,24,3000);
}\
