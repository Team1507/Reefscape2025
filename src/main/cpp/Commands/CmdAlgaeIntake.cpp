#include "commands/CmdAlgaeIntake.h"
#include "Robot.h"  
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

int algaeInriment;

CmdAlgaeIntake::CmdAlgaeIntake(double power)
  : m_power(power) 
  
{
  AddRequirements(&robotcontainer.m_algMotor);
}

void CmdAlgaeIntake::Initialize() {
  std::cout << "CmdAlgaeIntake::Initialize" << std::endl;
  m_timer.Reset();
  m_timer.Start();

  //Taken out of execute to prevent overwriting the scheduler
  // Force always into Intake Mode
  // m_mode = Mode::Intake;
  std::cout << "CmdAlgaeIntake: Mode set to Intake" << std::endl;
  if (robotcontainer.m_algMotor.GetAlgaePhotoEye()) {
    robotcontainer.m_algMotor.SetIntakePower(-0.2);  // Creep intake
    std::cout << "CmdAlgaeIntake: Algae detected, creeping intake" << std::endl;
  } else {
    robotcontainer.m_algMotor.SetIntakePower(-0.9);  // Full speed intake
    std::cout << "CmdAlgaeIntake: Full power intake" << std::endl;
  }
}

void CmdAlgaeIntake::Execute() {}

void CmdAlgaeIntake::End(bool interrupted) {
  std::cout << "CmdAlgaeIntake::End" << std::endl;
  robotcontainer.m_algMotor.SetIntakePower(-0.05); //Holding Power Adjust as needed
  m_timer.Stop();
}

bool CmdAlgaeIntake::IsFinished() {
  // Finish when the ball is detected
  if (robotcontainer.m_algMotor.GetAlgaePhotoEye()) {
    if(algaeInriment > 1)
    {    
      std::cout << "CmdAlgaeIntake: Ball fully detected, intake finished" << std::endl;
      //  robotcontainer.m_algMotor.SetIntakePower(-0.05); //Holding Power Adjust as needed
      return true;
    }
    else
    {
      algaeInriment++;
    }

  }
  return false;
}
