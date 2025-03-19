#include "commands/CmdAlignToAprilTag.h"
#include "Robot.h"  
#include <cmath>

CmdAlignToAprilTag::CmdAlignToAprilTag(bool alignLeft)
    : m_alignLeft(alignLeft) {
  // Ensure that this command requires the drive subsystem.
  AddRequirements(&robotcontainer.driveSub);
}

void CmdAlignToAprilTag::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_targetAcquired = false;
  std::cout << "CmdAlignToAprilTag Initialized" << std::endl;
}

void CmdAlignToAprilTag::Execute() {
  // If we have not yet computed a target pose, try to get one from vision.
  if (!m_targetAcquired) {
    // Assume vision provides similar interface as the limelight.
    if (robotcontainer.vision.IsTargetValid()) { // <-- your vision system must support this method
      // Get the detected AprilTag pose.
      frc::Pose2d tagPose = robotcontainer.vision.GetTargetPose(); // <-- assumed method
      frc::Pose2d currentPose = robotcontainer.driveSub.GetRobotPose();
      
      // Compute the angle from the robot to the tag.
      double dx = tagPose.X().value() - currentPose.X().value();
      double dy = tagPose.Y().value() - currentPose.Y().value();
      double angleToTag = std::atan2(dy, dx); // in radians
      
      // Define hard-coded constants (adjust these as needed):
      constexpr double kOffset = 0.3;         // lateral offset in meters
      constexpr double kApproachDistance = 0.5; // distance from the tag to stop
      
      // Compute a perpendicular offset vector.
      double offsetX = 0.0;
      double offsetY = 0.0;
      if (m_alignLeft) {
        // Left offset: subtract sin, add cos
        offsetX = -std::sin(angleToTag) * kOffset;
        offsetY =  std::cos(angleToTag) * kOffset;
      } else {
        // Right offset: add sin, subtract cos
        offsetX =  std::sin(angleToTag) * kOffset;
        offsetY = -std::cos(angleToTag) * kOffset;
      }
      
      // Compute an approach vector so that the robot stops a fixed distance from the tag.
      double approachX = -std::cos(angleToTag) * kApproachDistance;
      double approachY = -std::sin(angleToTag) * kApproachDistance;
      
      // The target translation is the tag's position plus the approach vector and the lateral offset.
      frc::Translation2d targetTranslation(
          tagPose.X().value() + approachX + offsetX,
          tagPose.Y().value() + approachY + offsetY);
      
      // The target heading is chosen so that the robot faces the tag.
      frc::Rotation2d targetRotation(angleToTag);
      
      m_targetPose = frc::Pose2d(targetTranslation, targetRotation);
      m_targetAcquired = true;
      std::cout << "Target acquired, driving to: " 
                << m_targetPose.X().value() << ", " 
                << m_targetPose.Y().value() << std::endl;
    } else {
      // If no target is detected yet, drive slowly forward to search.
      frc::ChassisSpeeds searchSpeeds{};
      searchSpeeds.vx = -0.4;  // drive (backward) at a fixed speed
      searchSpeeds.vy = 0.0;
      searchSpeeds.omega = 0.0;
      robotcontainer.driveSub.Drive2(searchSpeeds);
      return;
    }
  }
  
  // Once a target pose has been acquired, use a simple proportional controller
  // (similar to your CmdDriveToPoint) to drive to that pose.
  frc::Pose2d currentPose = robotcontainer.driveSub.GetRobotPose();
  
  // Compute translational error.
  auto deltaX = m_targetPose.X() - currentPose.X();
  auto deltaY = m_targetPose.Y() - currentPose.Y();
  double distance = std::hypot(deltaX.value(), deltaY.value());
  
  // Compute heading error (in degrees, normalized to [-180,180]).
  double currentHeadingDeg = currentPose.Rotation().Degrees().value();
  double targetHeadingDeg = m_targetPose.Rotation().Degrees().value();
  double headingError = targetHeadingDeg - currentHeadingDeg;
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // Simple proportional gains (tweak these constants as needed).
  constexpr double kTransP = 1.0;
  constexpr double kRotP = 0.05;
  
  double speed = kTransP * distance;
  if (speed > 1.0) speed = 1.0;
  double omega = kRotP * headingError;
  
  // Compute direction unit vector.
  double ux = (distance > 0.001) ? deltaX.value() / distance : 0.0;
  double uy = (distance > 0.001) ? deltaY.value() / distance : 0.0;
  
  frc::ChassisSpeeds speeds{};
  speeds.vx = speed * ux;
  speeds.vy = speed * uy;
  speeds.omega = omega;
  
  robotcontainer.driveSub.Drive2(speeds);
  
  std::cout << "Driving: distance = " << distance
            << ", heading error = " << headingError << std::endl;
  
  // Optional: if the command takes too long, you could force finish or take other action.
  if (m_timer.HasElapsed(5.0_s)) {
    std::cout << "CmdAlignToAprilTag timed out" << std::endl;
  }
}

void CmdAlignToAprilTag::End(bool interrupted) {
  std::cout << "CmdAlignToAprilTag Ended" << std::endl;
  robotcontainer.driveSub.Stop();
}

bool CmdAlignToAprilTag::IsFinished() {
  if (m_targetAcquired) {
    frc::Pose2d currentPose = robotcontainer.driveSub.GetRobotPose();
    auto deltaX = m_targetPose.X() - currentPose.X();
    auto deltaY = m_targetPose.Y() - currentPose.Y();
    double distance = std::hypot(deltaX.value(), deltaY.value());
    return distance < 0.05; // finish when within 5 centimeters
  }
  return false;
}
