#include <iostream>
#include "commands/CmdClawOuttake.h"
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  
#include "Robot.h"

CmdClawOuttake::CmdClawOuttake(double power) 
{
  m_power = power;
  AddRequirements(&robotcontainer.m_claw);
}

void CmdClawOuttake::Initialize() 
{
  std::cout << "CmdClawOuttake has initialized" << std::endl;

  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer
}

void CmdClawOuttake::Execute() 
{
  robotcontainer.m_claw.SetClawPower(-1);

  
}

void CmdClawOuttake::End(bool interrupted) 
{
  std::cout << "CmdClawOuttake has ended" << std::endl;
  
  robotcontainer.m_claw.SetClawPower(0.0);
  robotcontainer.m_claw.isCoralReady = false;
  m_timer.Stop(); //Ends the timer

}

bool CmdClawOuttake::IsFinished()
{
  const units::second_t timeout = units::second_t(0.2);
  if(m_timer.Get() >= timeout)
  {
    return true;
  }
  else 
  {
    return false;
  }

}
