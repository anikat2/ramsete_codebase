#ifndef RAMSETE_H
#define RAMSETE_H

#include "main.h"
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
        void moveToPose(lemlib::Pose targPose);
    
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