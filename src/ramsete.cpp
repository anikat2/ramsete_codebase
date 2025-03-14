#include "main.h"
#include "ramsete.h"
#include <fstream>

RamseteController::RamseteController(double ib, double izeta)
    : b(ib), zeta(izeta) {}

void RamseteController::setTarget(double ix, double iy, double itheta,
                                  double ivel, double iomega) {
  double convFact = 0.0254;
  ix=ix;
  iy=iy;
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
  double omega = std::clamp(omegaDes + k * et + b * velDes * sinc_et * ey, -1.5, 1.5);

  double tolerance = 0.05; 
  output out;
  out.linVel = vel;
  out.angVel = omega;
  double minVelocity = 0.02;

  double distanceToTarget = std::hypot(desX - ix, desY - iy);

  if (distanceToTarget < tolerance && fabs(vel) < minVelocity && fabs(omega) < 0.05) {
      leftSide.move_velocity(0);
      rightSide.move_velocity(0);

      output out;
      out.linVel = 0;
      out.angVel = 0;
      return out;
  }

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
  if (fabs(linearVelocity) < 0.05) linearVelocity = 0;
  if (fabs(angularVelocity) < 0.05) angularVelocity = 0;
  static double prevVel = 0;  
  double maxAccel = 0.1;     
  linearVelocity = std::clamp(linearVelocity, prevVel - maxAccel, prevVel + maxAccel);
  prevVel = linearVelocity;    

  double wheelRpm = (linearVelocity * 60.0) / (M_PI * wheelDiameterMeters);
  
  double trackWidthMeters = drivetrain.trackWidth * 0.0254;
  double angularComponent = (angularVelocity * trackWidthMeters * 60.0) / (2 * M_PI * wheelDiameterMeters);
  
  double leftRpm = wheelRpm + angularComponent;
  double rightRpm = wheelRpm - angularComponent;
  
  double maxRpm = 600; 
  leftRpm = std::clamp(leftRpm, -maxRpm, maxRpm);
  rightRpm = std::clamp(rightRpm, -maxRpm, maxRpm);
  pros::lcd::set_text(2,"left rpm" + std::to_string(leftRpm));
  pros::lcd::set_text(2, "right rpm" + std::to_string(rightRpm));
  leftSide.move_velocity(leftRpm);
  rightSide.move_velocity(rightRpm);
}

void RamseteController::moveToPose(lemlib::Pose targPose) {
  lemlib::Pose startPose = chassis.getPose();

  double distanceToTarget = std::hypot(
      targPose.x - startPose.x, 
      targPose.y - startPose.y
  );

  // 🔥 Use Trapezoidal Motion Profile Instead of Sigmoid
  SigmoidMotionProfile profile(
      distanceToTarget, 
      30.0,   // Max velocity
      30.0,
      20.0   // Acceleration
  );

  double totalTime = profile.get_time_end();
  double dt = 0.01;  // 10ms update interval

  double startHeading = startPose.theta * (M_PI / 180.0);
  double targetHeading = targPose.theta * (M_PI / 180.0);
  double headingDiff = targetHeading - startHeading;

  // Normalize heading difference
  while (headingDiff > M_PI) headingDiff -= 2 * M_PI;
  while (headingDiff < -M_PI) headingDiff += 2 * M_PI;

  for (double t = 0; t <= totalTime; t += dt) {
      double currentDistance = profile.get_time_distance(t);
      double currentVelocity = profile.get_time_velocity(t);

      double progress = (distanceToTarget > 0) ? currentDistance / distanceToTarget : 1.0;

      // 🔥 Gradually adjust heading using interpolation
      double currentHeading = startHeading + progress * headingDiff;
      double currentHeadingDeg = currentHeading * (180.0 / M_PI);

      double currentX = startPose.x + progress * (targPose.x - startPose.x);
      double currentY = startPose.y + progress * (targPose.y - startPose.y);

      // 🔥 Smoothly interpolate angular velocity based on heading error
      double headingError = targetHeading - currentHeading;
      while (headingError > M_PI) headingError -= 2 * M_PI;
      while (headingError < -M_PI) headingError += 2 * M_PI;

      double targetAngularVelocity = headingError / dt;
      targetAngularVelocity = std::clamp(targetAngularVelocity, -1.5, 1.5);

      setTarget(
          currentX,
          currentY,
          currentHeadingDeg,
          currentVelocity,
          targetAngularVelocity // 🔥 Now robot can turn gradually
      );

      lemlib::Pose currentPose = chassis.getPose(true);

      double distanceRemaining = std::hypot(
          targPose.x - currentPose.x, 
          targPose.y - currentPose.y
      );

      // Stop when close enough to target
      if (distanceRemaining < 0.05 && std::abs(headingError) < 2.0) {
          leftSide.move_velocity(0);
          rightSide.move_velocity(0);
          break;
      }

      auto output = step(currentPose);
      setMotorVoltages(output.linVel, output.angVel);

      pros::delay(static_cast<int>(dt * 1000));
  }

  leftSide.move_velocity(0);
  rightSide.move_velocity(0);
}
