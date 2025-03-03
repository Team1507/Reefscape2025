#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>


// --- SUBSYSTEMS ---
#include "subsystems/Claw.h"
#include "subsystems/Climber.h"
#include "Subsystems/Elevator.h"
#include "str/vision/VisionSystem.h"
#include "Subsystems/Drive.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  Climber m_climber;
  Claw    m_claw;
  Elevator m_elevator;

  str::vision::VisionSystem& GetVision();
  Drive& GetDrive();


  frc2::CommandXboxController m_topDriver{1};


 private:
  void ConfigureBindings();
  
  bool SmartDashHoming;

  void ConfigureSysIdBinds();
  frc2::CommandPtr SteerVoltsSysIdCommands(std::function<bool()> fwd,
                                           std::function<bool()> quasistatic);
  frc2::CommandPtr SteerTorqueCurrentSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr DriveSysIdCommands(std::function<bool()> fwd,
                                      std::function<bool()> quasistatic);
  frc2::CommandPtr WheelRadiusSysIdCommands(std::function<bool()> fwd);

  frc2::CommandXboxController driverJoystick{0};

   Drive driveSub{};

  str::vision::VisionSystem vision{
      [this](const frc::Pose2d& pose, units::second_t time,
             const Eigen::Vector3d& stdDevs) {
        driveSub.AddVisionMeasurement(pose, time, stdDevs);
      },
      [this](const frc::Pose2d& pose, units::second_t time,
             const Eigen::Vector3d& stdDevs) {
        driveSub.AddSingleTagVisionMeasurement(pose, time, stdDevs);
      }};

  std::shared_ptr<nt::NetworkTable> tuningTable{
      nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
  frc2::NetworkButton steerSysIdVoltsBtn{tuningTable, "SteerSysIdVolts"};
  frc2::NetworkButton steerSysIdTorqueCurrentBtn{tuningTable,
                                                 "SteerSysIdTorqueCurrent"};
  frc2::NetworkButton driveSysIdBtn{tuningTable, "DriveSysId"};
  frc2::NetworkButton wheelRadiusBtn{tuningTable, "WheelRadius"};



};
