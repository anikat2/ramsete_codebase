#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

#include "main.h"

class RamseteController {
public:
    struct output {
        double linVel;
        double angVel;
    };

    RamseteController(double ib, double izeta);

    void setTarget(double ix, double iy, double itheta, double ivel, double iomega);

    output step(double ix, double iy, double itheta);

    output step(lemlib::Pose ipose);

    void setGains(double ib, double izeta);

private:
    double b;            
    double zeta;         
    double desX, desY;   
    double desT;        
    double velDes;       
    double omegaDes;     
};

#endif 
