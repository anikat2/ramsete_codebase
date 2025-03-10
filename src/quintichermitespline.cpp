#include "main.h"
#include <cmath>

QuinticHermiteSpline::QuinticHermiteSpline(lemlib::Pose pose0, lemlib::Pose pose1,
                                           double velocity0, double velocity1)
    : pose0(pose0), pose1(pose1), velocity0(velocity0), velocity1(velocity1) {}

lemlib::Pose QuinticHermiteSpline::getPose(double t) const {
    if (t < 0) t = 0;
    if (t > 1) t = 1;

    double h0Val = h0(t);
    double h1Val = h1(t);
    double h4Val = h4(t);
    double h5Val = h5(t);

    double x = h0Val * pose0.x + h1Val * velocity0 * cos(pose0.theta) +
               h4Val * velocity1 * cos(pose1.theta) + h5Val * pose1.x;
    double y = h0Val * pose0.y + h1Val * velocity0 * sin(pose0.theta) +
               h4Val * velocity1 * sin(pose1.theta) + h5Val * pose1.y;

    double theta = pose0.theta + t * (pose1.theta - pose0.theta);

    return {x, y, theta};
}

SplineOutput QuinticHermiteSpline::getVelocityOutput(double t) const {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    
    double dh0 = -30*pow(t,4) + 60*pow(t,3) - 30*pow(t,2);
    double dh1 = -15*pow(t,4) + 32*pow(t,3) - 18*pow(t,2) + 1;
    double dh4 = -15*pow(t,4) + 28*pow(t,3) - 12*pow(t,2);
    double dh5 = 30*pow(t,4) - 60*pow(t,3) + 30*pow(t,2);
    
    double dx_dt = dh0 * pose0.x + dh1 * velocity0 * cos(pose0.theta) +
                   dh4 * velocity1 * cos(pose1.theta) + dh5 * pose1.x;
    double dy_dt = dh0 * pose0.y + dh1 * velocity0 * sin(pose0.theta) +
                   dh4 * velocity1 * sin(pose1.theta) + dh5 * pose1.y;
    
    double linearVelocity = std::hypot(dx_dt, dy_dt);
    
    double angularVelocity = (pose1.theta - pose0.theta);
    
    return {linearVelocity, angularVelocity};
}

double QuinticHermiteSpline::h0(double t) const {
    return -6 * pow(t, 5) + 15 * pow(t, 4) - 10 * pow(t, 3) + 1;
}

double QuinticHermiteSpline::h1(double t) const {
    return -3 * pow(t, 5) + 8 * pow(t, 4) - 6 * pow(t, 3) + t;
}

double QuinticHermiteSpline::h2(double t) const {
    return -0.5 * (pow(t, 5)) + 1.5 * pow(t, 4) - 1.5 * pow(t, 3) + 0.5 * pow(t, 2);
}

double QuinticHermiteSpline::h3(double t) const {
    return 0.5 * pow(t, 5) - pow(t, 4) + 0.5 * pow(t, 3);
}

double QuinticHermiteSpline::h4(double t) const {
    return -3 * pow(t, 5) + 7 * pow(t, 4) - 4 * pow(t, 3);
}

double QuinticHermiteSpline::h5(double t) const {
    return 6 * pow(t, 5) - 15 * pow(t, 4) + 10 * pow(t, 3);
}