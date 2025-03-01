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
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  Climber m_climber;
  Claw    m_claw;
  Elevator m_elevator;

  str::vision::VisionSystem& GetVision();


  frc2::CommandXboxController m_topDriver{1};

   subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

 private:
  void ConfigureBindings();

   units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};
  
  bool SmartDashHoming;

    // str::vision::VisionSystem vision{
    //   [this](const frc::Pose2d& pose, units::second_t time,
    //          const Eigen::Vector3d& stdDevs) {
    //     driveSub.AddVisionMeasurement(pose, time, stdDevs);
    //   },
    //   [this](const frc::Pose2d& pose, units::second_t time,
    //          const Eigen::Vector3d& stdDevs) {
    //     driveSub.AddSingleTagVisionMeasurement(pose, time, stdDevs);
    //   }};

  // std::shared_ptr<nt::NetworkTable> tuningTable{
  //     nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  // frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  // frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
  // frc2::NetworkButton steerSysIdVoltsBtn{tuningTable, "SteerSysIdVolts"};
  // frc2::NetworkButton steerSysIdTorqueCurrentBtn{tuningTable,
  //                                                "SteerSysIdTorqueCurrent"};
  // frc2::NetworkButton driveSysIdBtn{tuningTable, "DriveSysId"};
  // frc2::NetworkButton wheelRadiusBtn{tuningTable, "WheelRadius"};



};
