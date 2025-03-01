
#include "Commands/CmdAlgaeSetPosition.h"
#include "Robot.h"
#include <iostream>

CmdAlgaeSetPosition::CmdAlgaeSetPosition(double position) 
{
  m_position = position;
}

void CmdAlgaeSetPosition::Initialize()
{
  std::cout << "CmdElevatorSetPosition " << m_position << std::endl;
  robotcontainer.m_claw.SetPosition( m_position );
}

