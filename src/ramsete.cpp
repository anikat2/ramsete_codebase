#include "main.h"
#include "ramsete.h"
#include <fstream>

RamseteController::RamseteController(double ib, double izeta)
    : b(ib), zeta(izeta) {}

void RamseteController::setTarget(double ix, double iy, double itheta,
                                  double ivel, double iomega) {
  double convFact = 0.0254;
  desX = ix * convFact;
  desY = iy * convFact;

  desT = itheta * (M_PI / 180.0);
  
  convFact = M_PI * drivetrain.wheelDiameter * 0.0254 / 60.0; 
  velDes = ivel * convFact;

  convFact = 2 * M_PI / 60.0;
  omegaDes = iomega * convFact;
}

RamseteController::output RamseteController::step(double ix, double iy, double itheta) {
  double convFact = 0.0254;
  double x = ix * convFact;
  double y = iy * convFact;

  double theta = itheta * (M_PI / 180.0);
  
  double ex = desX - x;
  double ey = desY - y;
  double et = desT - theta;
  
  while (et > M_PI) et -= 2 * M_PI;
  while (et < -M_PI) et += 2 * M_PI;

  double ct = theta;
  double tempEx = cos(ct) * ex + sin(ct) * ey;
  double tempEy = -sin(ct) * ex + cos(ct) * ey;
  ex = tempEx;
  ey = tempEy;

  double k = 2 * zeta * sqrt(omegaDes * omegaDes + b * velDes * velDes);
  double vel = velDes * cos(et) + k * ex;
  double sinc_et = (fabs(et) < 1e-3) ? (1 - et * et / 6) : sin(et) / et;
  double omega = omegaDes + k * et + b * velDes * sinc_et * ey;
  
  output out;
  out.linVel = vel;
  out.angVel = omega;
  return out;
}

RamseteController::output RamseteController::step(lemlib::Pose ipose) {
  return step(ipose.x, ipose.y, ipose.theta);
}

void RamseteController::setGains(double ib, double izeta) {
  if (ib > 0)
    b = ib;
  if (izeta > 0 && izeta <= 1)
    zeta = izeta;
}

void RamseteController::setMotorVoltages(double linearVelocity, double angularVelocity) {
  double wheelDiameterMeters = drivetrain.wheelDiameter * 0.0254;
  
  double wheelRpm = (linearVelocity * 60.0) / (M_PI * wheelDiameterMeters);
  
  double trackWidthMeters = drivetrain.trackWidth * 0.0254;
  double angularComponent = (angularVelocity * trackWidthMeters * 60.0) / (2 * M_PI * wheelDiameterMeters);
  
  double leftRpm = wheelRpm + angularComponent;
  double rightRpm = wheelRpm - angularComponent;
  
  double maxRpm = 600; // Adjust based on motor specs
  leftRpm = std::clamp(leftRpm, -maxRpm, maxRpm);
  rightRpm = std::clamp(rightRpm, -maxRpm, maxRpm);
  leftSide.move_velocity(leftRpm);
  rightSide.move_velocity(rightRpm);

}

void RamseteController::moveToPose(lemlib::Pose targPose) {
  lemlib::Pose startPose = chassis.getPose();
  QuinticHermiteSpline spline(startPose, targPose, 200, 200);


  double totalTime = 5.0;
  double dt = 0.02;
  int steps = static_cast<int>(totalTime / dt);

  for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;

      lemlib::Pose targetPose = spline.getPose(t);
      
      double nextT = std::min(t + 0.01, 1.0);
      double targetVel = std::hypot(
          spline.getPose(nextT).x - spline.getPose(t).x,
          spline.getPose(nextT).y - spline.getPose(t).y
      ) / 0.01;
      
       
      double targetOmega = (spline.getPose(t + 0.01).theta - spline.getPose(t).theta) / 0.01;
    
      setTarget(targetPose.x, targetPose.y, targetPose.theta, targetVel, targetOmega);

      lemlib::Pose currentPose = chassis.getPose(true);

      auto output = step(currentPose);

      setMotorVoltages(output.linVel, output.angVel);

      pros::delay(static_cast<int>(dt * 1000));
  }
  
  leftSide.move_velocity(0);		
  rightSide.move_velocity(0);
}