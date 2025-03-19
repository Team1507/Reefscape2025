#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/Threads.h>
#include <frc2/command/CommandScheduler.h>
#include <wpinet/WebServer.h>

#include <ctre/phoenix6/SignalLogger.hpp>

#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "photon/PhotonPoseEstimator.h"

RobotContainer robotcontainer; // <--- global variable

void WriteToSmartDashboard(void);


Robot::Robot() {
  // // DANGEROUS MAKE SURE CODE DOESN'T BLOCK!!!
  frc::SetCurrentThreadPriority(true, 15);
  // ctre::phoenix6::SignalLogger::EnableAutoLogging(true);
  // ctre::phoenix6::SignalLogger::Start();
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  AddPeriodic([this] { robotcontainer.GetDrive().UpdateOdom(); },
              1 / consts::swerve::ODOM_UPDATE_RATE, 2_ms);
  wpi::WebServer::GetInstance().Start(5800,
                                      frc::filesystem::GetDeployDirectory());
  pdp.ClearStickyFaults();
  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic() {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastTotalLoopTime;
  loopTimePub.Set((1 / loopTime).value());

  WriteToSmartDashboard();

  frc2::CommandScheduler::GetInstance().Run();
  UpdateVision();

  lastTotalLoopTime = now;
  matchTimePub.Set(frc::DriverStation::GetMatchTime().value());
  battVoltagePub.Set(frc::RobotController::GetBatteryVoltage().value());

}

void Robot::SimulationPeriodic() {
  robotcontainer.GetVision().SimulationPeriodic(
      robotcontainer.GetDrive().GetOdomPose());
}

void Robot::UpdateVision() {
  auto robotPose = frc::Pose3d{robotcontainer.GetDrive().GetRobotPose()};
  robotcontainer.GetVision().UpdateYaws(robotcontainer.GetDrive().GetGyroYaw(),
                                     frc::Timer::GetFPGATimestamp());
  robotcontainer.GetVision().UpdatePoseEstimators(robotPose);
  robotcontainer.GetVision().UpdateCameraPositionVis(robotPose);
}

void Robot::DisabledInit() {
  // m_container.GetPivot().SetToStartingPosition();
  // m_container.GetElevator().SetToZeroHeight();
}

void Robot::DisabledPeriodic() {
  // m_container.GetPivot().SetToStartingPosition();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = robotcontainer.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

void WriteToSmartDashboard()
{
    frc::SmartDashboard::PutNumber("Odo X" , robotcontainer.driveSub.GetRobotPose().X().value());
    frc::SmartDashboard::PutNumber("Odo Y" , robotcontainer.driveSub.GetRobotPose().Y().value());
    frc::SmartDashboard::PutNumber("Odo Angle" , robotcontainer.driveSub.GetRobotPose().Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber("Vision X" , robotcontainer.vision.GetDetectedX());
    frc::SmartDashboard::PutNumber("Vision Y" , robotcontainer.vision.GetDetectedY());
}