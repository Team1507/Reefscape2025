#include "Commands/CmdPivotToPos.h"
#include "Robot.h"
#include <iostream>

CmdPivotToPos::CmdPivotToPos(float position) 
{
  AddRequirements(&robotcontainer.m_pivot);
  m_position = position;
}

void CmdPivotToPos::Initialize() 
{
  if (robotcontainer.m_elevator.isElevatorClearForPivot())
  {
    std::cout << "Pivot to position: " << m_position << std::endl;
    robotcontainer.m_pivot.SetTargetPosition(m_position);
  }
}
