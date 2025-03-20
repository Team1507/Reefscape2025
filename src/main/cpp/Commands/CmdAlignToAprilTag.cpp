#include "commands/CmdAlignToAprilTag.h"
#include "Robot.h"  
#include <cmath>  // For math functions like atan2, hypot
#include <units/velocity.h>  // WPILib units for velocity
#include <units/angular_velocity.h>  // WPILib units for angular velocity

// Simplify unit namespace usage
using namespace units;
using namespace units::velocity;
using namespace units::angular_velocity;

#define PI 3.1415

// Constructor: Sets alignment side and subsystem requirements
CmdAlignToAprilTag::CmdAlignToAprilTag(bool alignLeft)
    : m_alignLeft(alignLeft) {  // Store alignment direction (left/right)
  AddRequirements(&robotcontainer.driveSub);  // Requires exclusive drive subsystem control
}

// Initialization: Reset state when command starts
void CmdAlignToAprilTag::Initialize() {
  m_timer.Reset();  // Reset execution timer
  m_timer.Start();  // Start tracking command duration
  m_targetAcquired = false;  // Reset target lock flag
  std::cout << "CmdAlignToAprilTag Initialized" << std::endl;
}

// Main execution loop: Called repeatedly while command is active
void CmdAlignToAprilTag::Execute() {
  // TARGET ACQUISITION PHASE
  if (!m_targetAcquired) {
    // Use LimeLight instead of the old vision system.
    if (robotcontainer.m_limelight2.IsTargetValid()) {  // If LimeLight reports a valid target
      // Get the current robot pose
      frc::Pose2d currentPose = robotcontainer.driveSub.GetRobotPose();
      
      // Retrieve LimeLight data
      double tx = robotcontainer.m_limelight2.GetTargetHAngle();  // Horizontal offset (degrees)
      double distance = robotcontainer.m_limelight2.GetTargetDistance();  // Assumed to be properly implemented
      
      // Convert tx to radians and compute absolute angle to tag:
      double txRad = tx * (PI / 180.0);
      double angleToTag = currentPose.Rotation().Radians().value() + txRad;
      std::cout << "tx (deg): " << tx 
                << ", angleToTag (rad): " << angleToTag << std::endl;
      
      // Compute the estimated AprilTag pose based on current robot pose, distance, and tx
      double tagX = currentPose.X().value() + distance * std::cos(angleToTag);
      double tagY = currentPose.Y().value() + distance * std::sin(angleToTag);
      frc::Pose2d tagPose{
          frc::Translation2d(units::meter_t(tagX), units::meter_t(tagY)),
          frc::Rotation2d(units::radian_t(angleToTag))
      };
      std::cout << "Estimated Tag Pose: (" << tagX << ", " << tagY << ")" << std::endl;
      
      // Configuration constants
      constexpr double kOffset = 0.3;         // Lateral offset from tag (meters)
      constexpr double kApproachDistance = 0.5; // Standoff distance from tag (meters)
      
      // Calculate lateral offset (left/right of tag)
      double offsetX = 0.0;
      double offsetY = 0.0;
      if (m_alignLeft) {
        // Perpendicular offset calculation (-sinθ, cosθ)
        offsetX = -std::sin(angleToTag) * kOffset;
        offsetY =  std::cos(angleToTag) * kOffset;
      } else {
        // Opposite perpendicular offset (sinθ, -cosθ)
        offsetX =  std::sin(angleToTag) * kOffset;
        offsetY = -std::cos(angleToTag) * kOffset;
      }
      std::cout << "offsetX: " << offsetX << ", offsetY: " << offsetY << std::endl;
      
      // Calculate approach offset (move back from tag)
      double approachX = -std::cos(angleToTag) * kApproachDistance;
      double approachY = -std::sin(angleToTag) * kApproachDistance;
      std::cout << "approachX: " << approachX << ", approachY: " << approachY << std::endl;
      
      // Combine offsets to create final target position
      frc::Translation2d targetTranslation(
          units::meter_t(tagPose.X().value() + approachX + offsetX),
          units::meter_t(tagPose.Y().value() + approachY + offsetY));
      
      // Target orientation (face directly towards the tag)
      frc::Rotation2d targetRotation = frc::Rotation2d(units::radian_t(angleToTag));
      
      // Store final target pose
      m_targetPose = frc::Pose2d(targetTranslation, targetRotation);
      m_targetAcquired = true;  // Lock target coordinates
      std::cout << "Target acquired, driving to: " 
                << m_targetPose.X().value() << ", " 
                << m_targetPose.Y().value() << std::endl;
    } else {
      // SEARCH PATTERN: Drive backward slowly if no target
      frc::ChassisSpeeds searchSpeeds;
      searchSpeeds.vx = meters_per_second_t(-0.4);  // Reverse at 0.4 m/s
      searchSpeeds.vy = meters_per_second_t(0.0);     // No lateral movement
      searchSpeeds.omega = radians_per_second_t(0.0);// No rotation
      robotcontainer.driveSub.Drive2(searchSpeeds);
      std::cout << "No target detected, executing search pattern." << std::endl;
      return;  // Skip rest of execution until target found
    }
  }
  
  // MOVEMENT EXECUTION PHASE
  // Calculate position error
  frc::Pose2d currentPose = robotcontainer.driveSub.GetRobotPose();
  auto deltaX = m_targetPose.X() - currentPose.X();
  auto deltaY = m_targetPose.Y() - currentPose.Y();
  double distance = std::hypot(deltaX.value(), deltaY.value());  // Straight-line distance
  std::cout << "Distance to target: " << distance << std::endl;
  
  // Calculate heading error (wrapped to [-180, 180] degrees)
  double currentHeadingDeg = currentPose.Rotation().Degrees().value();
  double targetHeadingDeg = m_targetPose.Rotation().Degrees().value();
  double headingError = targetHeadingDeg - currentHeadingDeg;
  headingError = std::fmod(headingError + 180.0, 360.0) - 180.0;  // Wrap error
  std::cout << "Current heading: " << currentHeadingDeg 
            << ", Target heading: " << targetHeadingDeg 
            << ", Heading error: " << headingError << std::endl;
  
  // Control constants (tune these for robot performance)
  constexpr double kTransP = 1.0;  // Translation proportional gain
  constexpr double kRotP = 0.05;   // Rotation proportional gain
  
  // Calculate control outputs
  double speed = kTransP * distance;  // Speed proportional to distance
  speed = std::clamp(speed, -1.0, 1.0);  // Limit maximum speed
  double omega = kRotP * headingError;  // Angular velocity proportional to error
  std::cout << "Calculated speed: " << speed 
            << ", Angular velocity: " << omega << std::endl;
  
  // Calculate direction unit vector
  double ux = (distance > 0.001) ? deltaX.value() / distance : 0.0;
  double uy = (distance > 0.001) ? deltaY.value() / distance : 0.0;
  std::cout << "Unit vector (ux, uy): " << ux << ", " << uy << std::endl;
  
  // Convert to chassis speeds
  frc::ChassisSpeeds speeds;
  speeds.vx = meters_per_second_t(speed * ux);  // X velocity component
  speeds.vy = meters_per_second_t(speed * uy);    // Y velocity component
  speeds.omega = radians_per_second_t(omega);     // Rotation speed
  
  // Send drive command
  robotcontainer.driveSub.Drive2(speeds);
  std::cout << "Driving: distance = " << distance
            << ", heading error = " << headingError << std::endl;
  
  // Safety timeout check
  if (m_timer.HasElapsed(5_s)) {
    std::cout << "CmdAlignToAprilTag timed out" << std::endl;
  }
}

// Cleanup: Called when command ends
void CmdAlignToAprilTag::End(bool interrupted) {
  std::cout << "CmdAlignToAprilTag Ended" << std::endl;
  // Stop all drive motors by sending zero speeds
  robotcontainer.driveSub.Drive2(frc::ChassisSpeeds{}); 
}

// Completion check: Called every cycle to determine if command should finish
bool CmdAlignToAprilTag::IsFinished() {
  if (m_targetAcquired) {
    // Check proximity to target
    frc::Pose2d currentPose = robotcontainer.driveSub.GetRobotPose();
    auto deltaX = m_targetPose.X() - currentPose.X();
    auto deltaY = m_targetPose.Y() - currentPose.Y();
    double distance = std::hypot(deltaX.value(), deltaY.value());
    std::cout << "Checking if finished: current distance = " << distance << std::endl;
    return distance < 0.05;  // 5 cm position tolerance
  }
  return false;  // Continue until target reached or timeout
}
