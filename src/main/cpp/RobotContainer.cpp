#include "RobotContainer.h"

///---Commands---
#include <frc2/command/Commands.h>
#include "commands/CmdClimberActivate.h"
#include "commands/CmdClawActivate.h"
#include "commands/CmdClawOuttake.h"
#include "commands/CmdAlgaeOuttake.h"
#include "commands/CmdAlgaeIntake.h"
#include "commands/CmdAlgaeSetPosition.h"
#include "Commands/CmdElevatorPosition.h"
#include "Commands/CmdElevatorHome.h"
#include "Commands/CmdElevatorManualPower.h"
#include "Commands/CmdAlgaeManualPower.h"
#include "Commands/CmdPivotZero.h"
#include "Commands/CmdElevatorToPosition.h"

#include "Subsystems/Elevator.h"
#include "Subsystems/Claw.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  
#include "Robot.h"


#include <frc/MathUtil.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <cstddef>


#include "frc/RobotBase.h"
#include "frc/filter/Debouncer.h"
#include "frc2/command/sysid/SysIdRoutine.h"

#include "str/DriverstationUtils.h"


RobotContainer::RobotContainer() 
{
  ConfigureBindings();

  m_elevator.SetDefaultCommand(CmdElevatorManualPower(0));
  m_claw.SetDefaultCommand(CmdAlgaeManualPower(0));
  
  frc::SmartDashboard::PutData("zero pivot", new CmdPivotZero());
}

void RobotContainer::ConfigureBindings()
{

      // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    // (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    // (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    // (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // // reset the field-centric heading on left bumper press
    joystick.A().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
    

  //Climber
  m_topDriver.Back().WhileTrue(new CmdClimberActivate(frc::SmartDashboard::PutNumber("Climber Power", 0.35)));

  //Coral
  //m_topDriver.RightBumper().WhileTrue(new CmdClawOuttake(frc::SmartDashboard::PutNumber("ClawOut Power", -0.9)));
  joystick.RightBumper().OnTrue(new CmdClawActivate(-1.0));
  
  //Algae
  //m_topDriver.LeftBumper().WhileTrue(new CmdAlgaeOuttake(frc::SmartDashboard::PutNumber("AlgaeOut Power", 1)));
  joystick.LeftBumper().OnTrue(new CmdAlgaeIntake(0));
  m_topDriver.B().WhileTrue(new CmdAlgaeSetPosition(15));

  // m_topDriver.Y().ToggleOnTrue(new CmdPivotAngle(0.0, 0.0)); //Change Later
  // m_topDriver.Y().ToggleOnFalse(new CmdPivotAngle(0.0, 0.0)); //Change Later

  if(!m_topDriver.Y().Get())
  {
    m_topDriver.POVDown().OnTrue(new CmdElevatorToPosition(1));
    m_topDriver.POVRight().OnTrue(new CmdElevatorToPosition(2));
    m_topDriver.POVLeft().OnTrue(new CmdElevatorToPosition(3));
    m_topDriver.POVUp().OnTrue(new CmdElevatorToPosition(4));
  }
  else
  {
    m_topDriver.POVUp().OnTrue(new CmdElevatorToPosition(5));
    m_topDriver.POVDown().OnTrue(new CmdElevatorToPosition(6));
    m_topDriver.POVLeft().OnTrue(new CmdElevatorHome());
  }
  //m_topDriver.Back().WhileFalse(new CmdAlgaeManualPower(0));


}

frc2::Command* RobotContainer::GetAutonomousCommand() 
{
  return nullptr;
}

// void RobotContainer::ConfigureSysIdBinds() {
//   tuningTable->PutBoolean("SteerPidTuning", false);
//   tuningTable->PutBoolean("DrivePidTuning", false);
//   tuningTable->PutBoolean("SteerSysIdVolts", false);
//   tuningTable->PutBoolean("SteerSysIdTorqueCurrent", false);
//   tuningTable->PutBoolean("DriveSysId", false);
//   tuningTable->PutBoolean("WheelRadius", false);

//   steerTuneBtn.OnTrue(
//       driveSub.TuneSteerPID([this] { return !steerTuneBtn.Get(); }));
//   driveTuneBtn.OnTrue(
//       driveSub.TuneDrivePID([this] { return !driveTuneBtn.Get(); }));
//   steerSysIdVoltsBtn.WhileTrue(SteerVoltsSysIdCommands(
//       [this] { return tuningTable->GetBoolean("Forward", true); },
//       [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

//   steerSysIdTorqueCurrentBtn.WhileTrue(SteerTorqueCurrentSysIdCommands(
//       [this] { return tuningTable->GetBoolean("Forward", true); },
//       [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

// }

// frc2::CommandPtr RobotContainer::SteerVoltsSysIdCommands(
//     std::function<bool()> fwd, std::function<bool()> quasistatic) {
//   return frc2::cmd::Either(
//       frc2::cmd::Either(
//           driveSub.SysIdSteerQuasistaticVoltage(
//               frc2::sysid::Direction::kForward),
//           driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kForward),
//           quasistatic),
//       frc2::cmd::Either(
//           driveSub.SysIdSteerQuasistaticVoltage(
//               frc2::sysid::Direction::kReverse),
//           driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kReverse),
//           quasistatic),
//       fwd);
// }

// frc2::CommandPtr RobotContainer::SteerTorqueCurrentSysIdCommands(
//     std::function<bool()> fwd, std::function<bool()> quasistatic) {
//   return frc2::cmd::Either(
//       frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
//                             frc2::sysid::Direction::kForward),
//                         driveSub.SysIdSteerDynamicTorqueCurrent(
//                             frc2::sysid::Direction::kForward),
//                         quasistatic),
//       frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
//                             frc2::sysid::Direction::kReverse),
//                         driveSub.SysIdSteerDynamicTorqueCurrent(
//                             frc2::sysid::Direction::kReverse),
//                         quasistatic),
//       fwd);
// }

// frc2::CommandPtr RobotContainer::DriveSysIdCommands(
//     std::function<bool()> fwd, std::function<bool()> quasistatic) {
//   return frc2::cmd::Either(
//       frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
//                             frc2::sysid::Direction::kForward),
//                         driveSub.SysIdDriveDynamicTorqueCurrent(
//                             frc2::sysid::Direction::kForward),
//                         quasistatic),
//       frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
//                             frc2::sysid::Direction::kReverse),
//                         driveSub.SysIdDriveDynamicTorqueCurrent(
//                             frc2::sysid::Direction::kReverse),
//                         quasistatic),
//       fwd);
// }

// frc2::CommandPtr RobotContainer::WheelRadiusSysIdCommands(
//     std::function<bool()> fwd) {
//   return frc2::cmd::Either(
//       driveSub.WheelRadius(frc2::sysid::Direction::kForward),
//       driveSub.WheelRadius(frc2::sysid::Direction::kReverse), fwd);
// }

// Drive& RobotContainer::GetDrive() {
//   return driveSub;
// }



// str::vision::VisionSystem& RobotContainer::GetVision() {
//   return vision;
// }


