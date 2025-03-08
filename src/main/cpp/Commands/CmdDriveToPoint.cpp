#include "commands/CmdDriveToPoint.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angular_velocity.h>
#include <iostream>

CmdDriveToPoint::CmdDriveToPoint(units::meter_t x, units::meter_t y, units::degree_t heading,
                               units::meters_per_second_t velocity, bool stop, units::second_t timeout)
    : m_finalX(x), m_finalY(y), m_finalH(heading), 
      m_velocity(velocity), m_stop(stop), m_timeout(timeout) {
  AddRequirements(&robotcontainer.driveSub);
}

void CmdDriveToPoint::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  
  auto currentPose = robotcontainer.driveSub.GetRobotPose();
  m_startDistance = units::math::hypot(
      m_finalX - currentPose.X(),
      m_finalY - currentPose.Y()
  );
}

void CmdDriveToPoint::Execute() {
  constexpr auto ROT_kP = 5 / 1_s;  // 100 rad/s per rad error (≈ 1.745 rad/s per degree)

  auto currentPose = robotcontainer.driveSub.GetRobotPose();
  units::degree_t currentHeading = robotcontainer.driveSub.GetYaw();

  // Calculate translational components
  units::meter_t deltaX = m_finalX - currentPose.X();
  units::meter_t deltaY = m_finalY - currentPose.Y();
  units::meter_t distance = units::math::hypot(deltaX, deltaY);

  // Calculate heading error with wrapping
  units::degree_t headingError = m_finalH - currentHeading;
  headingError = units::math::fmod(headingError + 180_deg, 360_deg) - 180_deg;

  // Convert to radians for angular velocity calculation
  units::radians_per_second_t omega = headingError * ROT_kP;

  // Calculate direction vector
  double ux = 0.0;
  double uy = 0.0;
  if (distance > 0.05_m) {
      ux = deltaX.value() / distance.value();
      uy = deltaY.value() / distance.value();
      std::cout << "calculate direction vector" << std::endl;
  }

  // Calculate speeds
  frc::ChassisSpeeds speeds;
  if (distance > 0.05_m) {
      speeds.vx = m_velocity * ux;
      speeds.vy = m_velocity * uy;
      std::cout << "calculate speeds" << std::endl;
  }
  
  // Set angular velocity
  speeds.omega = omega;

  robotcontainer.driveSub.Drive2(
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          speeds.vx, 
          speeds.vy, 
          speeds.omega,
          currentPose.Rotation()
      )
  );
}

void CmdDriveToPoint::End(bool interrupted) {
  if(m_stop) {
      robotcontainer.driveSub.Drive2(frc::ChassisSpeeds{});
  }
  std::cout << "CmdDriveToPoint End" << std::endl;
}

bool CmdDriveToPoint::IsFinished() {
  auto currentPose = robotcontainer.driveSub.GetRobotPose();
  units::meter_t distance = units::math::hypot(
      m_finalX - currentPose.X(),
      m_finalY - currentPose.Y()
  );

  return (distance < 0.05_m) || m_timer.HasElapsed(m_timeout);
  
}