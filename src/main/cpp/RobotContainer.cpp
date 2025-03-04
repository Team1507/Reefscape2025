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
#include "Commands/CmdRampDrop.h"
#include  "Commands/CmdAlgaeSetPosition.h"

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
#include <frc2/command/ParallelCommandGroup.h>


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

    joystick.RightTrigger(0.5).WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * CreepSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * CreepSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * CreepAngularRate); // Drive counterclockwise with negative X (left)
        }));

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
  //m_topDriver.Y().WhileTrue(new CmdClimberActivate(frc::SmartDashboard::PutNumber("Climber Power", 0.35)));
  m_topDriver.B() && m_topDriver.Back().OnTrue(new CmdRampDrop());
  m_topDriver.Start() && m_topDriver.Y().OnTrue(new CmdClimberActivate(0.5));


  //Coral
  joystick.RightBumper().WhileTrue(new CmdClawOuttake(-1.0));
  m_topDriver.RightTrigger(0.5).OnTrue(new CmdClawActivate(-1.0));
  
  //Algae
  //m_topDriver.LeftBumper().WhileTrue(new CmdAlgaeOuttake(frc::SmartDashboard::PutNumber("AlgaeOut Power", 1)));
  joystick.LeftBumper().OnTrue(new CmdAlgaeOuttake(1.0));
  m_topDriver.LeftTrigger(0.5).OnTrue(new CmdAlgaeIntake(-1.0));
  

  // m_topDriver.Y().ToggleOnTrue(new CmdPivotAngle(0.0, 0.0)); //Change Later
  // m_topDriver.Y().ToggleOnFalse(new CmdPivotAngle(0.0, 0.0)); //Change Later


  // Create button objects (assuming your m_topDriver returns button objects)
auto aButton = m_topDriver.A();
auto povUpButton = m_topDriver.POVUp();
auto povDownButton = m_topDriver.POVDown();
auto povLeftButton = m_topDriver.POVLeft();
auto povRightButton = m_topDriver.POVRight();

frc2::Trigger altPovUp([&]() {
  return aButton.Get() && povUpButton.Get();
});
altPovUp.OnTrue(new CmdElevatorToPosition(ELEV_POS_L1));

frc2::Trigger altPovDown([&]() {
  return aButton.Get() && povDownButton.Get();
});
altPovDown.OnTrue(new CmdElevatorToPosition(ELEV_POS_HOME));
                                                 
frc2::Trigger altPovLeft([&]() {
  return aButton.Get() && povLeftButton.Get();
});
altPovLeft.OnTrue(new frc2::ParallelCommandGroup( CmdElevatorToPosition(ELEV_POS_ALG_LOW),
                                                  CmdAlgaeSetPosition(0))); //Change Later

frc2::Trigger altPovRight([&]() {
  return aButton.Get() && povRightButton.Get();
});
altPovRight.OnTrue(new frc2::ParallelCommandGroup( CmdElevatorToPosition(ELEV_POS_ALG_HIGH),
                                                  CmdAlgaeSetPosition(0))); //Change Later

// Normal D-pad bindings when A is not pressed
povUpButton.OnTrue(new CmdElevatorToPosition(ELEV_POS_L4));
povDownButton.OnTrue(new CmdElevatorToPosition(ELEV_POS_HOME));
povLeftButton.OnTrue(new CmdElevatorToPosition(ELEV_POS_L3));
povRightButton.OnTrue(new CmdElevatorToPosition(ELEV_POS_L2));


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


