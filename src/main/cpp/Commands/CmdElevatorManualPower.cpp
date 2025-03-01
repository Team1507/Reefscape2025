#include "Commands/CmdElevatorManualPower.h"
#include "Robot.h"
#include <iostream>
#include "constants/Constants.h"

#define ELEVATOR_DEADBAND_CONSTANT 0.3

CmdElevatorManualPower::CmdElevatorManualPower(float power) 
{
  AddRequirements(&robotcontainer.m_elevator);
  m_power = power;
  m_manualElevatorEnabled = false;
}

void CmdElevatorManualPower::Initialize() 
{
  std::cout << "Starting Elevator Manual Power" << std::endl;
}

void CmdElevatorManualPower::Execute() 
{
    if((robotcontainer.m_topDriver.GetLeftY() < -ELEVATOR_DEADBAND_CONSTANT))
  {
    robotcontainer.m_elevator.SetElevatorCoast();
    robotcontainer.m_elevator.SetElevatorPower(-0.25);
    m_manualElevatorEnabled = true;
  }
  else if(robotcontainer.m_topDriver.GetLeftY() > ELEVATOR_DEADBAND_CONSTANT)
  {
    robotcontainer.m_elevator.SetElevatorCoast();
    robotcontainer.m_elevator.SetElevatorPower(0.1);
    m_manualElevatorEnabled = true;
  }
  else if(m_manualElevatorEnabled)
  {
    robotcontainer.m_elevator.SetElvevatorBrake();
    robotcontainer.m_elevator.SetElevatorPower(-0.01);
    m_manualElevatorEnabled = false;
  }
}

void CmdElevatorManualPower::End(bool interrupted) 
{
  std::cout << "Ending Elevator Manual Power" << std::endl;
}

bool CmdElevatorManualPower::IsFinished()
{
  // if(m_manualElevatorEnabled)
  // {
  //   robotcontainer.m_elevator.SetElvevatorBrake();
  //   robotcontainer.m_elevator.SetElevatorPower(0);
  //   m_manualElevatorEnabled = false;
  //   return true;
  // }
  // else
  // {
  //   return false;
  // }
  return false;
}

