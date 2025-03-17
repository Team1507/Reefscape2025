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
void CmdAlgaeHome::Initialize() {
  std::cout << "Homing Algae" << std::endl;
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void CmdAlgaeHome::Execute() 
{
  robotcontainer.m_claw.SetPower(-0.1);
}

// Called once the command ends or is interrupted.
void CmdAlgaeHome::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdAlgaeHome::IsFinished() 
{

  const units::second_t timeout = units::second_t(1.0);
  if(m_timer.Get() < timeout)
  {
    robotcontainer.m_claw.SetPower(-0.1);
    return false;
  }
  else
  {
    std::cout << "No Homing Algae" << std::endl;
    robotcontainer.m_claw.SetPower(0);
    m_timer.Stop();
    return true;
  }

  
}
