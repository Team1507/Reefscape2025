#include "Commands/CmdElevatorPosition.h"
#include "Robot.h"
#include <iostream>

#include "Constants/Constants.h"

#include "Commands/CmdElevatorToPosition.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
CmdElevatorToPosition::CmdElevatorToPosition(float position) 
{
  AddRequirements(&robotcontainer.m_elevator);
  m_position = position; 
}

// Called when the command is initially scheduled.
void CmdElevatorToPosition::Initialize() 
{
  robotcontainer.m_elevator.SetTargetPosition(m_position);
}
