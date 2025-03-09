/**
 * RAMSETE Controller Implementation
 *
 * Best source for the math for this is:
 * https://file.tavsys.net/control/state-space-guide.pdf
 */
#include "main.h"
#include "ramsete.h"

RamseteController::RamseteController(double ib, double izeta)
    : b(ib), zeta(izeta) {}

void RamseteController::setTarget(double ix, double iy, double itheta,
                                  double ivel, double iomega) {
  // Inches to meters
  double convFact = 0.0254;
  desX = ix * convFact;
  desY = iy * convFact;

  // degrees to radians
  convFact = M_PI / 180.0;
  //desT = itheta * convFact;
  desT = itheta;
  // rpm to m/s
  convFact = M_PI * drivetrain.wheelDiameter / 60.0; 
  velDes = ivel * convFact;

  // rpm to rad/s
  convFact = 2 * M_PI / 60.0;
  omegaDes = iomega * convFact;
}

RamseteController::output RamseteController::step(double ix, double iy, double itheta) {
  // inches to meters
  double convFact = 0.0254;
  double ey = desX - ix * convFact;
  double ex = desY - iy * convFact;

  // degrees to radians
  convFact = M_PI / 180.0;
  //double et = desT - itheta * convFact;
  double et = desT-itheta;
  //double ct = M_PI / 2 - itheta * convFact;
  double ct = M_PI/2 - itheta;
  double tempEx = cos(-ct) * ex - sin(-ct) * ey;
  double tempEy = sin(-ct) * ex + cos(-ct) * ey;
  ex = tempEx;
  ey = tempEy;

  double k = 2 * zeta * sqrt(omegaDes * omegaDes + b * velDes * velDes);
  double vel = velDes * cos(et) + k * ex; 
  
  /*
  double curvature = (et != 0) ? sin(et) / et : 1.0;
  double omega = vel * curvature + k * et + b * velDes * sin(et) * ey / (et != 0 ? et : 1e-6);
  */

  double vel = velDes * cos(et) + k * ex; 
  double omega = omegaDes + k * et + b * velDes * sin(et) * ey / (et != 0 ? et : 1e-6);

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

void setMotorVoltages(double linearVelocity, double angularVelocity) {
  //inches to meters
  double diam = drivetrain.wheelDiameter/39.37;
  double leftVel = (linearVelocity/(M_PI*diam)) + angularVelocity;
  double rightVel = (linearVelocity/(M_PI*diam)) - angularVelocity;

  leftSide.move_velocity(leftVel);
  rightSide.move_velocity(rightVel);
}

void moveToPose(lemlib::Pose targPose) {
  lemlib::Pose startPose = chassis.getPose();
  QuinticHermiteSpline spline(startPose, targPose, 200, 200);
  double totalTime = 5.0;
  double dt = 0.02;
  int steps = static_cast<int>(totalTime / dt);

  RamseteController ramsete(2, 0.7);

  for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;

      lemlib::Pose targetPose = spline.getPose(t);

      ramsete.setTarget(targetPose.x, targetPose.y, targetPose.theta, drivetrain.rpm, drivetrain.rpm);

      lemlib::Pose currentPose = chassis.getPose(true);

      auto output = ramsete.step(currentPose);

      setMotorVoltages(output.linVel, output.angVel);

      pros::delay(static_cast<int>(dt * 1000));
  }
}
