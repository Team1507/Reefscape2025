#pragma once

#include "frc/geometry/Pose2d.h"
#include "str/vision/Camera.h"
#include "units/angle.h"

namespace str::vision {
class VisionSystem {
 public:
  explicit VisionSystem(std::function<void(const frc::Pose2d&, units::second_t,
                                           const Eigen::Vector3d& stdDevs)>
                            visionConsumer,
                        std::function<void(const frc::Pose2d&, units::second_t,
                                           const Eigen::Vector3d& stdDevs)>
                            singleTagConsumer);
  void UpdateCameraPositionVis(frc::Pose3d robotPose);
  void SimulationPeriodic(frc::Pose2d simRobotPose);
  void UpdateYaws(units::radian_t yaw, units::second_t time) {
    for (auto& cam : cameras) {
      cam.AddYaw(yaw, time);
    }
  }
  void UpdatePoseEstimators(frc::Pose3d robotPose);

  bool IsTargetValid() const;
  frc::Pose2d GetTargetPose() const;

  double GetDetectedX() const;
  double GetDetectedY() const;
  int GetDetectedTagID() const;


 private:
  std::array<frc::Pose3d, 2> cameraLocations;
  nt::StructArrayPublisher<frc::Pose3d> cameraLocationsPub{
      nt::NetworkTableInstance::GetDefault()
          .GetTable("Vision")
          ->GetStructArrayTopic<frc::Pose3d>("CameraLocations")
          .Publish()};

  std::array<Camera, 2> cameras;

   std::optional<frc::Pose2d> m_lastDetectedPose;
};
}  // namespace str::vision