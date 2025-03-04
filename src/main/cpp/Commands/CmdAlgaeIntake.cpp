#include "commands/CmdAlgaeIntake.h"
#include "Robot.h"  
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

CmdAlgaeIntake::CmdAlgaeIntake(double power)
  : m_power(power) {
}

void CmdAlgaeIntake::Initialize() {
  std::cout << "CmdAlgaeIntake::Initialize" << std::endl;
  m_timer.Reset();
  m_timer.Start();

  //Taken out of execute to prevent overwriting the scheduler
  // Force always into Intake Mode
  m_mode = Mode::Intake;
  std::cout << "CmdAlgaeIntake: Mode set to Intake" << std::endl;
  if (robotcontainer.m_claw.GetAlgaePhotoEye()) {
    robotcontainer.m_claw.SetAlgaePower(-0.2);  // Creep intake
    std::cout << "CmdAlgaeIntake: Algae detected, creeping intake" << std::endl;
  } else {
    robotcontainer.m_claw.SetAlgaePower(-0.9);  // Full speed intake
    std::cout << "CmdAlgaeIntake: Full power intake" << std::endl;
  }
}

void CmdAlgaeIntake::Execute() {}

void CmdAlgaeIntake::End(bool interrupted) {
  std::cout << "CmdAlgaeIntake::End" << std::endl;
  robotcontainer.m_claw.SetAlgaePower(0.0);
  m_timer.Stop();

  if (robotcontainer.m_claw.GetAlgaePhotoEye()) {
    std::cout << "CmdAlgaeIntake: Un-jamming reverse" << std::endl;
    robotcontainer.m_claw.SetAlgaePower(-0.2);  // Small reverse
    frc::Wait(0.2_s);  // Wait 200ms
    robotcontainer.m_claw.SetAlgaePower(-0.05); //Holding Power Adjust as needed
    robotcontainer.m_claw.SetBallLoaded(true);
  }
}

bool CmdAlgaeIntake::IsFinished() {
  // Finish when the ball is detected
  if (robotcontainer.m_claw.GetAlgaePhotoEye()) {
    std::cout << "CmdAlgaeIntake: Ball fully detected, intake finished" << std::endl;
    return true;
  }
  return false;
}
