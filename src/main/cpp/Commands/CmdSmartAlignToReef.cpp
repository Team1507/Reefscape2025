// CmdSmartAlignToReef.cpp
#include "commands/CmdSmartAlignToReef.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <iostream>
#include "str/DriverstationUtils.h"

CmdSmartAlignToReef::CmdSmartAlignToReef(bool alignLeft,
                                       units::meters_per_second_t velocity,
                                       units::second_t timeout)
    : m_alignLeft(alignLeft), m_velocity(velocity), m_timeout(timeout) {
  AddRequirements(&robotcontainer.driveSub);
}

void CmdSmartAlignToReef::Initialize() {
  std::cout << "CmdSmartAlignToReef Init - Align Left: " 
            << m_alignLeft << std::endl;
  m_timer.Reset();
  m_timer.Start();
  
  // Get tunable offsets from SmartDashboard
  lOffset = units::inch_t{frc::SmartDashboard::GetNumber("L OFFSET", 0)};
  rOffset = units::inch_t{frc::SmartDashboard::GetNumber("R OFFSET", 0)};

  // Get target pose from predefined positions
  std::string targetKey = m_alignLeft ? "ReefLeft" : "ReefRight";
  m_targetPose = robotcontainer.driveSub.importantPoses.at(targetKey);

  // Apply alliance flip if needed
  if (str::IsOnRed()) {
    m_targetPose = pathplanner::FlippingUtil::flipFieldPose(m_targetPose);
  }

  // Apply side-specific offset
  if(m_alignLeft) {
    m_targetPose = m_targetPose.TransformBy(
        frc::Transform2d{0_m, lOffset, frc::Rotation2d{}});
  } else {
    m_targetPose = m_targetPose.TransformBy(
        frc::Transform2d{0_m, rOffset, frc::Rotation2d{}});
  }
}

void CmdSmartAlignToReef::Execute() {
  constexpr auto ROT_kP = 5 / 1_s;
  constexpr auto DECEL_DIST = 1.0_m;
  constexpr auto MIN_VEL = 0.3_mps;

  auto currentPose = robotcontainer.driveSub.GetRobotPose();
  
  // Calculate remaining distance
  units::meter_t deltaX = m_targetPose.X() - currentPose.X();
  units::meter_t deltaY = m_targetPose.Y() - currentPose.Y();
  units::meter_t distance = units::math::hypot(deltaX, deltaY);

  // Adaptive velocity with linear deceleration
  units::meters_per_second_t adaptiveVel = m_velocity;
  if(distance < DECEL_DIST) {
    adaptiveVel = m_velocity * (distance / DECEL_DIST);
    adaptiveVel = units::math::max(adaptiveVel, MIN_VEL);
  }

  // Calculate heading error
  units::degree_t targetHeading = m_targetPose.Rotation().Degrees();
  units::degree_t currentHeading = robotcontainer.driveSub.GetYaw();
  units::degree_t headingError = targetHeading - currentHeading;
  headingError = units::math::fmod(headingError + 180_deg, 360_deg) - 180_deg;

  // Calculate direction vector
  double ux = 0.0;
  double uy = 0.0;
  if(distance > 0.05_m) {
    ux = deltaX.value() / distance.value();
    uy = deltaY.value() / distance.value();
  }

  // Create chassis speeds
  frc::ChassisSpeeds speeds;
  speeds.vx = adaptiveVel * ux;
  speeds.vy = adaptiveVel * uy;
  speeds.omega = headingError * ROT_kP;

  // Drive field-relative
  robotcontainer.driveSub.Drive2(
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          speeds.vx, 
          speeds.vy, 
          speeds.omega,
          currentPose.Rotation()
      )
  );

  std::cout << "Aligning to " << (m_alignLeft ? "LEFT" : "RIGHT")
            << " | Distance: " << distance.value() 
            << "m | Vel: " << adaptiveVel.value() << "m/s" << std::endl;
}

void CmdSmartAlignToReef::End(bool interrupted) {
  robotcontainer.driveSub.Drive2(frc::ChassisSpeeds{});
  std::cout << "CmdSmartAlignToReef Ended" << std::endl;
}

bool CmdSmartAlignToReef::IsFinished() {
  auto currentPose = robotcontainer.driveSub.GetRobotPose();
  units::meter_t distance = units::math::hypot(
      m_targetPose.X() - currentPose.X(),
      m_targetPose.Y() - currentPose.Y()
  );
  return (distance < 0.05_m) || m_timer.HasElapsed(m_timeout);
}