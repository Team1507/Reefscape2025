
#include "commands/CmdAlgaeOuttake.h"
#include <iostream>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  
#include "Robot.h"

CmdAlgaeOuttake::CmdAlgaeOuttake(double power) 
{
  m_power = power;
  AddRequirements(&robotcontainer.m_claw);
}

void CmdAlgaeOuttake::Initialize() 
{
  std::cout << "Algae Outtake Init" << std::endl;
  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer

}

void CmdAlgaeOuttake::Execute() 
{
  robotcontainer.m_claw.SetClawPower(m_power);
}

void CmdAlgaeOuttake::End(bool interrupted) 
{
  std::cout << "CmdAlgaeOuttake has ended" << std::endl;
  
  robotcontainer.m_claw.SetClawPower(0.0);
  m_timer.Stop(); //Ends the timer
}

bool CmdAlgaeOuttake::IsFinished() 
{
  const units::second_t timeout = units::second_t(1.5);
  if(m_timer.Get() >= timeout)
  {
    return true;
    robotcontainer.m_claw.SetAlgaePower(0.0);
    robotcontainer.m_claw.SetPosition(0);
  }
  else 
  {
    return false;
  }
}
