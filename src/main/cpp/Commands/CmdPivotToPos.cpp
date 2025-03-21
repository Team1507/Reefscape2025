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
  robotcontainer.m_pivot.SetTargetPosition(m_position);
}
