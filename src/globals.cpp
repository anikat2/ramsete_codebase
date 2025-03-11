#include "main.h"

//pros::MotorGroup leftSide({4, 2, -3});
pros::MotorGroup leftSide({-4, -2, -3});

//pros::MotorGroup rightSide({11, 12, -13}); 
pros::MotorGroup rightSide({15, 14, 13});
pros::Rotation rotation(16);
pros::Motor intake(20);
pros::adi::DigitalOut clamp('A'); 
pros::adi::DigitalOut doinker('B'); 

pros::MotorGroup lb({-11, 12});
lemlib::Drivetrain drivetrain(&leftSide, // left motor group
    &rightSide, // right motor group
    10.75, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    450, // drivetrain rpm is 360
    4 // horizontal drift is 2 (for now)
);
pros::IMU imu(18);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);
/*
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              44.5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              15 // maximum acceleration (slew)
);
*/
lemlib::ControllerSettings lateral_controller(45, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              70, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// Create the angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                           0.01, // small integral gain (kI)
                                           5, // derivative gain (kD)
                                           1.0, // anti windup
                                           1.5, // small error range, in degrees
                                           100, // small error range timeout, in milliseconds
                                           3.0, // large error range, in degrees
                                           250, // large error range timeout, in milliseconds
                                           3.0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
lateral_controller, // lateral PID settings
angular_controller, // angular PID settings
sensors // odometry sensors
);
