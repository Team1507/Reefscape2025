#include "Commands/CmdElevatorPosition.h"
#include "Robot.h"
#include <iostream>

#include "Constants/Constants.h"

CmdElevatorPosition::CmdElevatorPosition(float position) 
{
  AddRequirements(&robotcontainer.m_elevator);
  m_position = position; 
}

void CmdElevatorPosition::Initialize() 
{
  std::cout << "Starting Elevator Position" << std::endl; 
}

void CmdElevatorPosition::Execute() 
{
  robotcontainer.m_elevator.SetTargetPosition(m_position);
}

void CmdElevatorPosition::End(bool interrupted) 
{
  std::cout << "Ending Elevator Position" << std::endl;
}


bool CmdElevatorPosition::IsFinished()
{
  // if((robotcontainer.m_elevator.GetElevatorPosition() + ELEV_TOLERANCE) > m_position && (robotcontainer.m_elevator.GetElevatorPosition() - ELEV_TOLERANCE) < m_position)
  // if(robotcontainer.m_elevator.GetElevatorPosition() <= -1)
  // {
    // robotcontainer.m_elevator.SetElvevatorBrake();
    // robotcontainer.m_elevator.SetElevatorPower(-0.025);
  //   return true;
  // }
  // else
  // {
    //return false;
  //}

  if(robotcontainer.m_elevator.AtSetpoint()) {return true;}
  else {return false;}
}


