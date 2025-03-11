#include "main.h"
void drive() {
    // loop forever
    
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // move the robot
        
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
void togglePiston() {
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
