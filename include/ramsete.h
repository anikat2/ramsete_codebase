#ifndef RAMSETE_H
#define RAMSETE_H

#include "main.h"

class QuinticHermiteSpline {
public:
    QuinticHermiteSpline(lemlib::Pose pose0, lemlib::Pose pose1,
                        double velocity0, double velocity1);
    
    lemlib::Pose getPose(double t) const;
    
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

class RamseteController {
public:
    struct output {
        double linVel;
        double angVel;
    };

    RamseteController(double b, double zeta);
    
    void setTarget(double x, double y, double theta, double vel, double omega);
    
    output step(double x, double y, double theta);
    output step(lemlib::Pose pose);
    
    void setGains(double b, double zeta);
    
    void setMotorVoltages(double linearVelocity, double angularVelocity);
    
    void moveToPose(lemlib::Pose targetPose);

private:
    double b;
    double zeta;
    
    double desX;
    double desY;
    double desT;
    double velDes;
    double omegaDes;
};


#endif