#ifndef QUINTIC_HERMITE_SPLINE_H
#define QUINTIC_HERMITE_SPLINE_H

#include "main.h"

struct SplineOutput {
    double linearVelocity;
    double angularVelocity;
};

class QuinticHermiteSpline {
public:
    QuinticHermiteSpline(lemlib::Pose pose0, lemlib::Pose pose1,
                         double velocity0, double velocity1);

    lemlib::Pose getPose(double t) const;
    SplineOutput getVelocityOutput(double t) const;

private:
    lemlib::Pose pose0;
    lemlib::Pose pose1;
    double velocity0;
    double velocity1;

    double h0(double t) const;
    double h1(double t) const;
    double h2(double t) const;
    double h3(double t) const;
    double h4(double t) const;
    double h5(double t) const;
};

#endif 