#include "main.h"

//pros::MotorGroup leftSide({4, 2, -3});
pros::MotorGroup leftSide({-4, -2, -3});

//pros::MotorGroup rightSide({11, 12, -13}); 
pros::MotorGroup rightSide({11, 12, 13});

lemlib::Drivetrain drivetrain(&leftSide, // left motor group
    &rightSide, // right motor group
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    450, // drivetrain rpm is 360
    2 // horizontal drift is 2 (for now)
);
pros::IMU imu(20);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(8.4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Create the angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              19.78, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
lateral_controller, // lateral PID settings
angular_controller, // angular PID settings
sensors // odometry sensors
);
