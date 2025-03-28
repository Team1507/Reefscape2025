#include "RobotContainer.h"

///---Commands---
#include <frc2/command/Commands.h>
#include "commands/CmdClimberActivate.h"
#include "commands/CmdClawActivate.h"
#include "commands/CmdClawOuttake.h"
#include "commands/CmdAlgaeOuttake.h"
#include "commands/CmdAlgaeIntake.h"
#include "Commands/CmdElevatorPosition.h"
#include "Commands/CmdElevatorHome.h"
#include "Commands/CmdElevatorManualPower.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdRampDrop.h"
#include "Commands/CmdDriveClearAll.h"
#include "Commands/CmdAlignToAprilTag.h"
#include "Commands/CmdPivotZero.h"

#include "Subsystems/Elevator.h"
#include "Subsystems/Claw.h"
#include "Subsystems/Climber.h"
#include "Subsystems/Drive.h"
#include "str/vision/VisionSystem.h"

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

#include "commands/AutoDoNothing.h"
#include "commands/Auto2PieceRightBlue.h"
#include "commands/Auto2PieceRightRed.h"
#include "commands/Auto2PieceLeft.h"
#include "Commands/Auto3PieceRight.h"
#include "Commands/Auto1PieceRight.h"
#include "Commands/AutoMoveForward.h"
#include "Commands/Auto1PieceMiddle.h"
#include "Commands/Auto1PieceMiddleAlg.h"
#include "Commands/CmdClawStop.h"
#include "Commands/CmdPivotManual.h"
#include "Commands/CmdPivotToPos.h"


RobotContainer::RobotContainer() 
{
  ConfigureBindings();

  m_elevator.SetDefaultCommand(CmdElevatorManualPower(0));
  m_pivot.SetDefaultCommand(CmdPivotManual());
  

    m_chooser.AddOption("Auto Do Nothing", new AutoDoNothing() );

    m_chooser.SetDefaultOption("Auto Do Nothing", new AutoDoNothing() );

    m_chooser.AddOption("Auto 2 Piece Right Blue", new Auto2PieceRightBlue());

     m_chooser.AddOption("Auto 2 Piece Right Red", new Auto2PieceRightRed());

    m_chooser.AddOption("Auto 2 Piece Left", new Auto2PieceLeft());

    m_chooser.AddOption("Auto 1 Piece Middle" , new Auto1PieceMiddle());

    m_chooser.AddOption("Auto 1 Piece Middle" , new Auto1PieceMiddleAlg());

    m_chooser.AddOption("Auto Move Forward" , new AutoMoveForward());

  frc::SmartDashboard::PutData("Auto Mode", &m_chooser);


  frc::SmartDashboard::PutData("Zero Pivot", new CmdPivotZero());

}

void RobotContainer::ConfigureBindings()
{
   driveSub.SetDefaultCommand(driveSub.DriveTeleop(
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftY(), .025) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftX(), .025) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverJoystick.GetRightX(), .025) *
               360_deg_per_s;
      }));

  driverJoystick.RightTrigger(0.5).WhileTrue(robotcontainer.driveSub.DriveTeleop([this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftY(), .025) *
           consts::swerve::physical::DRIVE_CREEP_SPEED);
      },
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftX(), .025) *
            consts::swerve::physical::DRIVE_CREEP_SPEED);
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverJoystick.GetRightX(), .025) *
               200_deg_per_s;
      }));

  driverJoystick.X().WhileTrue(new CmdAlignToAprilTag(true));
  driverJoystick.B().WhileTrue(new CmdAlignToAprilTag(false));

  //Climber
  //m_topDriver.Y().WhileTrue(new CmdClimberActivate(frc::SmartDashboard::PutNumber("Climber Power", 0.35)));
  (m_topDriver.B() && m_topDriver.Back()).OnTrue(new CmdRampDrop());
  (m_topDriver.Y() && m_topDriver.Start()).OnTrue(new CmdClimberActivate(0.7, 0.68));


  //Coral
   driverJoystick.RightBumper().WhileTrue(new CmdClawOuttake(-1.0));
  m_topDriver.RightTrigger(0.5).OnTrue(new CmdClawActivate(-1.0));
  driverJoystick.LeftTrigger(0.5).WhileTrue(new CmdClawStop());
  
  //Algae
  //m_topDriver.LeftBumper().WhileTrue(new CmdAlgaeOuttake(frc::SmartDashboard::PutNumber("AlgaeOut Power", 1)));
  driverJoystick.LeftBumper().OnTrue(new CmdAlgaeOuttake(1.0));
  m_topDriver.LeftTrigger(0.5).OnTrue(new CmdAlgaeIntake(-1.0));
  driverJoystick.POVDown().OnTrue(new CmdPivotZero());
  
 driverJoystick.A().OnTrue(new CmdDriveClearAll());


// Assume these button objects are stored persistently (here as local constants)
const auto aButton       = m_topDriver.A();
const auto povUpButton   = m_topDriver.POVUp();
const auto povDownButton = m_topDriver.POVDown();
const auto povLeftButton = m_topDriver.POVLeft();
const auto povRightButton= m_topDriver.POVRight();

// Alternate bindings (only fire when A is pressed)
frc2::Trigger altPovUp([=]() {
  return aButton.Get() && povUpButton.Get();
});
altPovUp.OnTrue(new CmdElevatorToPosition(ELEV_POS_L1));

frc2::Trigger altPovDown([=]() {
  return aButton.Get() && povDownButton.Get();
});
altPovDown.OnTrue(new CmdElevatorToPosition(ELEV_POS_HOME));

frc2::Trigger altPovLeft([=]() {
  return aButton.Get() && povLeftButton.Get();
});
altPovLeft.OnTrue(new frc2::ParallelCommandGroup( CmdElevatorToPosition(ELEV_POS_ALG_HIGH), CmdPivotToPos(2), CmdAlgaeIntake(-1.0)));

frc2::Trigger altPovRight([=]() {
  return aButton.Get() && povRightButton.Get();
});
altPovRight.OnTrue(new frc2::ParallelCommandGroup( CmdElevatorToPosition(ELEV_POS_ALG_LOW), CmdPivotToPos(2), CmdAlgaeIntake(-1.0)));

// Normal D-pad bindings (fire only when A is NOT pressed)
frc2::Trigger normalPovUp([=]() {
  return !aButton.Get() && povUpButton.Get();
});
normalPovUp.OnTrue(new CmdElevatorToPosition(ELEV_POS_L4));

frc2::Trigger normalPovDown([=]() {
  return !aButton.Get() && povDownButton.Get();
});
normalPovDown.OnTrue(new frc2::ParallelCommandGroup( CmdElevatorToPosition(ELEV_POS_HOME) , CmdPivotToPos(1)));

frc2::Trigger normalPovLeft([=]() {
  return !aButton.Get() && povLeftButton.Get();
});
normalPovLeft.OnTrue(new CmdElevatorToPosition(ELEV_POS_L3));

frc2::Trigger normalPovRight([=]() {
  return !aButton.Get() && povRightButton.Get();
});
normalPovRight.OnTrue(new CmdElevatorToPosition(ELEV_POS_L2));

m_topDriver.RightBumper().OnTrue(new CmdPivotToPos(2));
m_topDriver.LeftBumper().OnTrue(new CmdPivotToPos(1));

}

void RobotContainer::ConfigureSysIdBinds() {
  tuningTable->PutBoolean("SteerPidTuning", false);
  tuningTable->PutBoolean("DrivePidTuning", false);
  tuningTable->PutBoolean("SteerSysIdVolts", false);
  tuningTable->PutBoolean("SteerSysIdTorqueCurrent", false);
  tuningTable->PutBoolean("DriveSysId", false);
  tuningTable->PutBoolean("WheelRadius", false);

 steerTuneBtn.OnTrue(
      driveSub.TuneSteerPID([this] { return !steerTuneBtn.Get(); }));
  driveTuneBtn.OnTrue(
      driveSub.TuneDrivePID([this] { return !driveTuneBtn.Get(); }));

 steerSysIdVoltsBtn.WhileTrue(SteerVoltsSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  steerSysIdTorqueCurrentBtn.WhileTrue(SteerTorqueCurrentSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  driveSysIdBtn.WhileTrue(DriveSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  wheelRadiusBtn.WhileTrue(WheelRadiusSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); }));

}
frc2::CommandPtr RobotContainer::SteerVoltsSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kForward),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kForward),
          quasistatic),
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kReverse),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kReverse),
          quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::SteerTorqueCurrentSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        driveSub.SysIdSteerDynamicTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        driveSub.SysIdSteerDynamicTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::DriveSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        driveSub.SysIdDriveDynamicTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        driveSub.SysIdDriveDynamicTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::WheelRadiusSysIdCommands(
    std::function<bool()> fwd) {
  return frc2::cmd::Either(
      driveSub.WheelRadius(frc2::sysid::Direction::kForward),
      driveSub.WheelRadius(frc2::sysid::Direction::kReverse), fwd);
}

frc2::Command* RobotContainer::GetAutonomousCommand() 
{
  return m_chooser.GetSelected();
}
Drive& RobotContainer::GetDrive() {
  return driveSub;
}

str::vision::VisionSystem& RobotContainer::GetVision() {
  return vision;
}

Claw& RobotContainer::GetClaw() {
  return m_claw;
}

Climber& RobotContainer::GetClimber() {
  return m_climber;
}

Elevator& RobotContainer::GetElevator() {
  return m_elevator;
} 


