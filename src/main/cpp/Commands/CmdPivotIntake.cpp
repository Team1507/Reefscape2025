// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/CmdPivotIntake.h"
#include "Robot.h"
#include "Constants/Constants.h"
#include "constants/Presets.h"
#include <iostream>
#include <cmath>
#include <units/angle.h>


CmdPivotIntake::CmdPivotIntake(double power) {

 // AddRequirements(&robotcontainer.m_pivot);
  m_power = power;
}

// Called when the command is initially scheduled.
void CmdPivotIntake::Initialize() 
{
  std::cout << "CmdPivotIntake has initialized" << std::endl;
  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer

  robotcontainer.m_pivot.SetIntakePower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void CmdPivotIntake::Execute() {}

// Called once the command ends or is interrupted.
void CmdPivotIntake::End(bool interrupted) 
{
  std::cout << "CmdPivotIntake has ended" << std::endl;
  robotcontainer.m_pivot.SetIntakePower(0); //Stop the intake
  m_timer.Stop(); //Stop the timer
}

// Returns true when the command should end.
bool CmdPivotIntake::IsFinished() {
   const units::second_t timeout = units::second_t(0.5);
  if(m_timer.Get() >= timeout)
  {
    return true;
  }
    return false;
}
