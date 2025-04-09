// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/CmdUnDropRamp.h"
#include "Robot.h"

CmdUnDropRamp::CmdUnDropRamp() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdUnDropRamp::Initialize()
{
  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer
}

// Called repeatedly when this Command is scheduled to run
void CmdUnDropRamp::Execute() 
{
  robotcontainer.m_climber.ResetRamp();
}

// Called once the command ends or is interrupted.
void CmdUnDropRamp::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdUnDropRamp::IsFinished() 
{
  const units::second_t timeout = units::second_t(0.25);
  if(m_timer.Get() >= timeout)
  {
    robotcontainer.m_climber.OffRamp();
    return true;
  }
  else 
  return false;
}
