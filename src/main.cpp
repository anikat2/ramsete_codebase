#include "main.h"

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
	pros::lcd::set_text(1, "Hello PROS User!");
    leftSide.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // Use COAST for smoother stops
    rightSide.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.calibrate();
	pros::delay(1000);
	pros::lcd::register_btn1_cb(on_center_button);
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
    RamseteController ramsete(2, 0.7);
    lemlib::Pose endPose{48, 48, 90};
	    // set position to x:0, y:0, heading:0
	chassis.setPose(0, 0, 0);
	//chassis.turnToHeading(90,1000);
    ramsete.moveToPose(endPose);
	

}	
/**
 * Runs the operator control code. lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);This function will be started in its own task
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
const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int auton = 177;
int zero = 0;
int states[numStates] = {0, 30, 150};
int currState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void liftControl() {
    double kp = 0.8;
    double error = target - (rotation.get_position()/100.0);
    double velocity = kp * error;
    lb.move(velocity);
} 
 void drive() {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        
        chassis.curvature(leftY , rightX);

        // delay to save resources
        pros::delay(25);
    
}
	bool rollerOnL = false;  
	bool buttonPressedL = false;
	bool rollerOnR = false;  
	bool buttonPressedR = false;
	void roller(){

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !buttonPressedL) {
				buttonPressedL = true; 

				rollerOnL = !rollerOnL; 

				if (rollerOnL) {
					intake.move(-110); // Start roller at 100 RPM
					
					
				} else {
					intake.move(0); // Stop the roller
					
				} 
		}

		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				buttonPressedL = false; 
		}



		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !buttonPressedR) {
				buttonPressedR = true; 

				rollerOnR = !rollerOnR; 

				if (rollerOnR) {
					intake.move(120); // Start roller at 100 RPM
					
					
				} else {
					intake.move(0); // Stop the roller
					
				} 
		}

			// Reset the buttonPressed flag when L1 is released
		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
				buttonPressedR = false; // Ready to detect next press
		}
		
		

	}
void toggleClamp() {
    static bool pistonState = true;  // Tracks the state of the piston
    static bool lastButtonState = false;  // Tracks the last state of the L2 button

    bool currentButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    // Toggle the piston state on a rising edge (button press)
    if (currentButtonState && !lastButtonState) {
        pistonState = !pistonState;  // Flip the state
        clamp.set_value(pistonState);  // Update the piston
        
    }

    lastButtonState = currentButtonState;  // Update the last button state
}
void toggleDoink() {
    static bool pistonState = true;  // Tracks the state of the piston
    static bool lastButtonState = false;  // Tracks the last state of the L2 button

    bool currentButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    // Toggle the piston state on a rising edge (button press)
    if (currentButtonState && !lastButtonState) {
        pistonState = !pistonState;  // Flip the state
        clamp.set_value(pistonState);  // Update the piston
        
    }

    lastButtonState = currentButtonState;  // Update the last button state
}
void opcontrol() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	rotation.reset_position();
	rotation.set_position(0);
	pros::Task liftControlTask([&]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });
	
	while (true) {
		pros::lcd::print(0, "OpControl Running");
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			nextState();
		}
		//controller.print(1, 0, "Arm Pos: %.2f", arm.get_position());
		controller.print(0, 0, "Rotation: %04d", (int)rotation.get_position());
		
		drive();
		roller();
		toggleClamp();
		toggleDoink();
	}
	
	

	
	
	pros::Task controllerTask([&]{
		while (true) {
			lemlib::Pose pose = chassis.getPose();
			//controller.print(0 ,0, "Angle: %f",  pose.theta);
			controller.print(1, 0, "Y: %f", pose.y);
			controller.print(2, 0, "X: %f", pose.x);
			controller.print(0, 0, "Rotation: %d", rotation.get_position());
		}
	});

//for testing purposes
//autonomous();
}