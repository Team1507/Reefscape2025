#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/length.h>
#include <units/angle.h>

#define BranchTrackKp 0.05
#define BranchTrackKi 0 
#define BranchTrackKd 0

void setLimelightNTDoubleArray(const std::string& limelightName, 
                               const std::string& key, 
                               const std::vector<double>& values);


class LimeLight : public frc2::SubsystemBase 
{
 public:
  // Constructor
  LimeLight(std::string llname);

  // LimeLight functions
  int    GetTargetId(void);
  bool   IsTargetValid(void);

  double GetTargetHAngle(void);
  double GetTargetVAngle(void);
  double GetTargetDistance(void);
  void   SetPipeline(int value);
  int    GetPipeline(void);

  void   SetLastSeenID(void);
  double GetTargetYaw(void);

  double TrackBranch(void);

  void   RunLimeLight(void);

  void Periodic() override;

  // Extra helper functions from additional header
  static inline frc::Pose3d toPose3D(const std::vector<double>& inData)
  {
      if(inData.size() < 6)
      {
          return frc::Pose3d();
      }
      return frc::Pose3d(
          frc::Translation3d(units::length::meter_t(inData[0]),
                             units::length::meter_t(inData[1]),
                             units::length::meter_t(inData[2])),
          frc::Rotation3d(units::angle::degree_t(inData[3]),
                          units::angle::degree_t(inData[4]),
                          units::angle::degree_t(inData[5])));
  }

  static inline frc::Pose2d toPose2D(const std::vector<double>& inData)
  {
      if(inData.size() < 6)
      {
          return frc::Pose2d();
      }
      return frc::Pose2d(
          frc::Translation2d(units::length::meter_t(inData[0]),
                             units::length::meter_t(inData[1])),
          frc::Rotation2d(units::angle::degree_t(inData[5])));
  }

  static inline void SetRobotOrientation(const std::string& limelightName, 
          double yaw, double yawRate, 
          double pitch, double pitchRate, 
          double roll, double rollRate)
  {
      std::vector<double> entries = {yaw, yawRate, pitch, pitchRate, roll, rollRate};
      // Calls a helper function to set the values in NetworkTables.
      // (Implementation is provided in LimeLight.cpp.)
      setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
  }

 private:
  int    m_lastSeenID{0};
  int    m_targetId{0};  
  bool   m_targetValid{false};
  double m_targetYaw{0.0};
  double m_targetDistance{0.0};
  std::string m_LLName;
  frc::PIDController m_positionBranchPID {BranchTrackKp, BranchTrackKi, BranchTrackKd};

  // Declare the helper function as friend (or you can put it in an anonymous namespace in the cpp)
  friend void setLimelightNTDoubleArray(const std::string& limelightName, const std::string& key, const std::vector<double>& values);
};
