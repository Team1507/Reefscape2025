#include "commands/CmdAlgaeIntake.h"
#include "Robot.h"  // This provides access to robotcontainer and m_claw.
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

CmdAlgaeIntake::CmdAlgaeIntake(double power)
  : m_power(power) {
  // Declare subsystem dependencies.
  AddRequirements(&robotcontainer.m_claw);
}

void CmdAlgaeIntake::Initialize() {
  std::cout << "CmdAlgaeIntake::Initialize" << std::endl;
  m_timer.Reset();
  m_timer.Start();

  // Determine the mode based on whether the ball is loaded.
  if (robotcontainer.m_claw.IsBallLoaded()) {
    m_mode = Mode::ShootOut;
    std::cout << "CmdAlgaeIntake: Mode set to ShootOut" << std::endl;
  } else {
    m_mode = Mode::Intake;
    std::cout << "CmdAlgaeIntake: Mode set to Intake" << std::endl;
  }
}

void CmdAlgaeIntake::Execute() {
  if (m_mode == Mode::Intake) {
    // In Intake mode, use the photeye to decide power.
    if (robotcontainer.m_claw.GetAlgaePhotoEye()) {
      robotcontainer.m_claw.SetAlgaePower(-0.2);
      std::cout << "CmdAlgaeIntake: Photeye triggered, creeping intake" << std::endl;
    } else {
      robotcontainer.m_claw.SetAlgaePower(-0.9);
      std::cout << "CmdAlgaeIntake: Intaking at full power" << std::endl;
    }
  } else {  // ShootOut mode
    robotcontainer.m_claw.SetAlgaePower(0.8);
    std::cout << "CmdAlgaeIntake: Shooting out ball" << std::endl;
  }
}

void CmdAlgaeIntake::End(bool interrupted) {
  std::cout << "CmdAlgaeIntake::End" << std::endl;
  m_timer.Stop();
  robotcontainer.m_claw.SetAlgaePower(0.0);

  // If we were in ShootOut mode, clear the ball-loaded flag.
  if (m_mode == Mode::ShootOut) {
    robotcontainer.m_claw.SetBallLoaded(false);
    std::cout << "CmdAlgaeIntake: Ball shot out, flag cleared" << std::endl;
  }
}

bool CmdAlgaeIntake::IsFinished() {
  if (m_mode == Mode::Intake) {
    // Finish when the photeye detects the ball.
    if (robotcontainer.m_claw.GetAlgaePhotoEye()) {
      robotcontainer.m_claw.SetBallLoaded(true);
      std::cout << "CmdAlgaeIntake: Ball loaded detected, finishing intake" << std::endl;
      return true;
    }
    return false;
  } else {  // ShootOut mode
    // Run the shoot-out for a fixed duration (1 second).
    const units::second_t shootOutDuration = units::second_t(1.0);
    return m_timer.Get() >= shootOutDuration;
  }
}
