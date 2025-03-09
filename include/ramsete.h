#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

#include "main.h"

class RamseteController {
    public:
        struct output {
            double linVel;
            double angVel;
        };
    
        RamseteController(double ib, double izeta) : b(ib), zeta(izeta) {}
    
        static void setTarget(double ix, double iy, double itheta, double ivel, double iomega);
        output step(double ix, double iy, double itheta);
        output step(double ix, double iy, double itheta);
        output step(lemlib::Pose ipose);
        void setGains(double ib, double izeta);
        void moveToPose(lemlib::Pose targPose);
            
    private:
        double b;
        double zeta;
        static double desX, desY, desT, velDes, omegaDes;
    };
    
    double RamseteController::desX = 0;
    double RamseteController::desY = 0;
    double RamseteController::desT = 0;
    double RamseteController::velDes = 0;
    double RamseteController::omegaDes = 0;
#endif    