// #include "commands/CmdSmartAlignToReef.h"
// #include <frc/geometry/Pose2d.h>
// #include <frc/smartdashboard/SmartDashboard.h>
// #include <units/math.h>
// #include <iostream>
// #include "Robot.h"

// CmdSmartAlignToReef::CmdSmartAlignToReef(bool alignLeft, units::meter_t customOffset)
//     : m_alignLeft(alignLeft), m_customOffset(customOffset) {
//   AddRequirements(&robotcontainer.driveSub);
// }

// void CmdSmartAlignToReef::Initialize() {
//   m_timer.Reset();
//   m_timer.Start();
//   m_targetPose = CalculateTargetPose();
//   m_lastDistance = units::math::hypot(
//       m_targetPose.X() - robotcontainer.driveSub.GetRobotPose().X(),
//       m_targetPose.Y() - robotcontainer.driveSub.GetRobotPose().Y()
//   );
// }

// frc::Pose2d CmdSmartAlignToReef::CalculateTargetPose() const {
//   // Get base target from existing reef alignment logic
//   auto basePose = robotcontainer.driveSub.importantPoses[
//       robotcontainer.driveSub.WhatPoleToGoTo(
//           robotcontainer.driveSub.WhatReefZoneAmIIn(), 
//           m_alignLeft
//       )
//   ];

//   // Apply offset based on alliance color
//   frc::Transform2d offset{0_m, m_customOffset, 0_deg};
//   if (str::IsOnRed()) {
//       basePose = pathplanner::FlippingUtil::flipFieldPose(basePose);
//       offset = frc::Transform2d{0_m, -m_customOffset, 0_deg};
//   }
  
//   return basePose.TransformBy(offset);
// }

// frc::Translation2d CmdSmartAlignToReef::GetOptimalApproachVector() const {
//   const auto currentPose = robotcontainer.driveSub.GetRobotPose();
//   const auto targetVec = m_targetPose.Translation() - currentPose.Translation();
//   const units::meter_t distance = targetVec.Norm();

//   // Calculate curvature based on distance
//   const double curvature = SmartAlignConstants::CURVATURE_FACTOR * 
//       (1.0 - units::math::min(distance / SmartAlignConstants::APPROACH_DISTANCE, 1.0));

//   // Create curved approach vector
//   return targetVec.Rotate(
//       m_alignLeft ? frc::Rotation2d(-curvature * 90_deg) 
//                  : frc::Rotation2d(curvature * 90_deg)
//   ).Normalize();
// }

// void CmdSmartAlignToReef::Execute() {
//   const auto currentPose = robotcontainer.driveSub.GetRobotPose();
//   const auto approachVector = GetOptimalApproachVector();
//   const units::meter_t distance = currentPose.Translation()
//       .Distance(m_targetPose.Translation());

//   // Calculate adaptive speeds
//   const units::meters_per_second_t speed = units::math::min(
//       SmartAlignConstants::MAX_SPEED * 
//       (distance / SmartAlignConstants::APPROACH_DISTANCE), 
//       SmartAlignConstants::MAX_SPEED
//   );

//   // Calculate heading error with continuous input
//   const units::radian_t headingError = units::math::fmod(
//       (m_targetPose.Rotation().Radians() - currentPose.Rotation().Radians()) + 
//       units::radian_t(std::numbers::pi), 
//       units::radian_t(2 * std::numbers::pi)
//   ) - units::radian_t(std::numbers::pi);

//   // Calculate angular velocity with clamping
//   const units::radians_per_second_t omega = units::math::clamp(
//       headingError * 3.0_rad_per_s, 
//       -SmartAlignConstants::MAX_ANGULAR_VEL, 
//       SmartAlignConstants::MAX_ANGULAR_VEL
//   );

//   // Combine translation and rotation
//   const frc::ChassisSpeeds speeds{
//       approachVector.X() * speed,
//       approachVector.Y() * speed,
//       omega
//   };

//   robotcontainer.driveSub.Drive2(
//       frc::ChassisSpeeds::FromFieldRelativeSpeeds(
//           speeds.vx, 
//           speeds.vy, 
//           speeds.omega,
//           currentPose.Rotation()
//       )
//   );

//   // Debugging
//   frc::SmartDashboard::PutNumber("Align/Speed", speed.value());
//   frc::SmartDashboard::PutNumber("Align/Omega", omega.value());
//   frc::SmartDashboard::PutNumber("Align/Distance", distance.value());
// }

// void CmdSmartAlignToReef::End(bool interrupted) {
//   robotcontainer.driveSub.Drive2(frc::ChassisSpeeds{});
// }

// bool CmdSmartAlignToReef::IsFinished() {
//   const auto currentPose = robotcontainer.driveSub.GetRobotPose();
//   const units::meter_t distance = currentPose.Translation()
//       .Distance(m_targetPose.Translation());
//   const units::radian_t headingError = units::math::abs(
//       currentPose.Rotation().Radians() - m_targetPose.Rotation().Radians()
//   );

//   // Dynamic completion conditions
//   const bool isStationary = (units::math::abs(m_lastDistance - distance) < 0.01_m);
//   const bool inPosition = distance < 0.1_m && headingError < 5_deg;
  
//   m_lastDistance = distance;
  
//   return inPosition || (isStationary && m_timer.HasElapsed(2_s)) || m_timer.HasElapsed(5_s);
// }