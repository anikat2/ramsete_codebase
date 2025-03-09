#include "main.h"

pros::MotorGroup leftSide({1, 2, 3});
pros::MotorGroup rightSide({4, 5, 6});

lemlib::Drivetrain drivetrain(&leftSide, // left motor group
    &rightSide, // right motor group
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    450, // drivetrain rpm is 360
    2 // horizontal drift is 2 (for now)
);
pros::IMU imu(12);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                      0, // integral gain (kI)
                      3, // derivative gain (kD)
                      3, // anti windup
                      1, // small error range, in inches
                      100, // small error range timeout, in milliseconds
                      3, // large error range, in inches
                      500, // large error range timeout, in milliseconds
                      20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                      0, // integral gain (kI)
                      10, // derivative gain (kD)
                      3, // anti windup
                      1, // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3, // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drive, // drivetrain settings
lateral_controller, // lateral PID settings
angular_controller, // angular PID settings
sensors // odometry sensors
);
