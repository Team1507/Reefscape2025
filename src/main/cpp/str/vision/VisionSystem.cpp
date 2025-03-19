#include "str/vision/VisionSystem.h"

#include <frc/geometry/Pose2d.h>

#include <vector>

#include "constants/VisionConstants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace str::vision;

VisionSystem::VisionSystem(
    std::function<void(const frc::Pose2d&, units::second_t,
                       const Eigen::Vector3d& stdDevs)>
        visionConsumer,
    std::function<void(const frc::Pose2d&, units::second_t,
                       const Eigen::Vector3d& stdDevs)>
        singleTagConsumer)
    : cameras{
          Camera{consts::vision::FL_CAM_NAME, consts::vision::FL_ROBOT_TO_CAM,
                 consts::vision::SINGLE_TAG_STD_DEV,
                 consts::vision::MULTI_TAG_STD_DEV, true, visionConsumer,
                 singleTagConsumer},
          Camera{consts::vision::FR_CAM_NAME, consts::vision::FR_ROBOT_TO_CAM,
                 consts::vision::SINGLE_TAG_STD_DEV,
                 consts::vision::MULTI_TAG_STD_DEV, true, visionConsumer,
                 singleTagConsumer}} {}

void VisionSystem::UpdateCameraPositionVis(frc::Pose3d robotPose) {
  cameraLocations[0] = robotPose.TransformBy(consts::vision::FL_ROBOT_TO_CAM);
  cameraLocations[1] = robotPose.TransformBy(consts::vision::FR_ROBOT_TO_CAM);

  cameraLocationsPub.Set(cameraLocations);
}

void VisionSystem::UpdatePoseEstimators(frc::Pose3d robotPose) {
  for (auto& cam : cameras) {
    cam.UpdatePoseEstimator(robotPose);
  
  
    auto maybePose = cam.GetLatestTagPose();
    if (maybePose.has_value()) {
      m_lastDetectedPose = maybePose.value().ToPose2d(); 
    }
  }
  }

void VisionSystem::SimulationPeriodic(frc::Pose2d simRobotPose) {
  for (auto& cam : cameras) {
    cam.SimPeriodic(simRobotPose);
  }
}

bool VisionSystem::IsTargetValid() const {
  return m_lastDetectedPose.has_value();
}

// New method to get the latest detected pose
frc::Pose2d VisionSystem::GetTargetPose() const {
  if (!m_lastDetectedPose.has_value()) {
    throw std::runtime_error("No target detected!");
  }
  return m_lastDetectedPose.value();
}

void VisionSystem::PublishTargetInfoToSmartDashboard() {
  bool targetFound = false;

  for (auto& cam : cameras) {
    auto allTargets = cam.GetAllTargets();  // Assuming this returns a list of detected targets
    if (!allTargets.empty()) {
      for (const auto& target : allTargets) {
        int targetID = target.GetFiducialId();
        auto targetTransform = target.GetBestCameraToTarget();
      frc::Pose2d targetPose{
      units::meter_t{targetTransform.X()},
      units::meter_t{targetTransform.Y()},
      frc::Rotation2d{targetTransform.Rotation().Z()}
};

        // Publish to SmartDashboard
        frc::SmartDashboard::PutNumber("Target ID", targetID);
        frc::SmartDashboard::PutNumber("Target X", targetPose.X().value());
        frc::SmartDashboard::PutNumber("Target Y", targetPose.Y().value());

        std::cout << "Target ID: " << targetID
                  << ", X: " << targetPose.X().value()
                  << ", Y: " << targetPose.Y().value() << std::endl;

        targetFound = true;
        break;  // Only display the first valid target
      }
    }
  }

  // If no targets were found, set default values
  if (!targetFound) {
    frc::SmartDashboard::PutNumber("Target ID", -1);
    frc::SmartDashboard::PutNumber("Target X", 0.0);
    frc::SmartDashboard::PutNumber("Target Y", 0.0);
  }
}