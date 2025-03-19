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

double VisionSystem::GetDetectedX() const {
  // Throws if no target is detected; alternatively, you could return NaN.
  if (!m_lastDetectedPose.has_value()) {
    throw std::runtime_error("No target detected!");
  }
  return m_lastDetectedPose.value().X().value();
}

double VisionSystem::GetDetectedY() const {
  if (!m_lastDetectedPose.has_value()) {
    throw std::runtime_error("No target detected!");
  }
  return m_lastDetectedPose.value().Y().value();
}

// int VisionSystem::GetDetectedTagID() const {
//   // This example assumes you have a method to retrieve the tag ID.
//   // If not available, return -1. Replace this with your actual logic.
//   if (!m_lastDetectedPose.has_value()) {
//     return -1;
//   }
//   // Example: use the first camera's tag ID if available.
//   // Replace GetLatestTagID() with your actual function.
//   int tagID = cam[0].GetLatestTagID();  
//   return tagID;
// }

