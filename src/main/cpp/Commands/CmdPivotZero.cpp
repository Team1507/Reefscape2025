// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/CmdPivotZero.h"
#include "Robot.h"

CmdPivotZero::CmdPivotZero() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdPivotZero::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdPivotZero::Execute() 
{
  robotcontainer.m_pivot.ResetEncoderValue();
}

// Called once the command ends or is interrupted.
void CmdPivotZero::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdPivotZero::IsFinished() {
  return true;
}
