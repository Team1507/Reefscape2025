// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/CmdAlgaeHome.h"
#include "Robot.h"
#include <iostream>

CmdAlgaeHome::CmdAlgaeHome() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdAlgaeHome::Initialize() 
{
  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer

  std::cout<< "Home Algea start" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void CmdAlgaeHome::Execute() 
{
  robotcontainer.m_claw.SetPower(0.3);
}

// Called once the command ends or is interrupted.
void CmdAlgaeHome::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdAlgaeHome::IsFinished() 
{
  
  if(m_timer.Get() > timeout)
  {
    std::cout<< "Home Algea end" << std::endl;
    robotcontainer.m_claw.SetPower(0.0);
    return true;
  }
  else
  {
    return false;
  }
  
}
