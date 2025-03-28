#include <iostream>
#include "commands/CmdClawOuttake.h"
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  
#include "Robot.h"

CmdClawOuttake::CmdClawOuttake(double power) 
{
  m_power = power;
}

void CmdClawOuttake::Initialize() 
{
  std::cout << "CmdClawOuttake has initialized" << std::endl;

  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer

   robotcontainer.m_claw.SetClawPower(-1); //Moved from execute, Should allow Pivot to change now - Trent
}

void CmdClawOuttake::Execute() 
{}

void CmdClawOuttake::End(bool interrupted) 
{
  std::cout << "CmdClawOuttake has ended" << std::endl;
  
  robotcontainer.m_claw.SetClawPower(0.0);
  robotcontainer.m_claw.isCoralReady = false;
  m_timer.Stop(); //Ends the timer

}

bool CmdClawOuttake::IsFinished()
{
  const units::second_t timeout = units::second_t(1.0);
  if(m_timer.Get() >= timeout)
  {
    return true;
  }
    return false;
  

}
