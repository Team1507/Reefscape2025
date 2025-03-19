// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/CmdClawStop.h"
#include "Robot.h"
#include "iostream"

CmdClawStop::CmdClawStop() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdClawStop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdClawStop::Execute() 
{
  robotcontainer.m_claw.m_clawStop = true;
  std::cout<<"STATE BREAKER 3000" << std::endl;

}

// Called once the command ends or is interrupted.
void CmdClawStop::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdClawStop::IsFinished() {
  return false;
}