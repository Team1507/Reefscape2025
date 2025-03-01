#include "Commands/CmdAlgaeManualPower.h"
#include "Robot.h"
#include <iostream>
#include "constants/Constants.h"

#define ALGAE_DEADBAND_CONSTANT 0.9

CmdAlgaeManualPower::CmdAlgaeManualPower(float power) 
{
  AddRequirements(&robotcontainer.m_claw);
  m_power = power;
  m_manualAlgaeEnabled = false;
}

void CmdAlgaeManualPower::Initialize() 
{
  std::cout << "Starting Algae Manual Power" << std::endl;
}

void CmdAlgaeManualPower::Execute() 
{
    if((robotcontainer.m_topDriver.GetRightY() > ALGAE_DEADBAND_CONSTANT && !m_manualAlgaeEnabled))
  {
    robotcontainer.m_claw.SetPower(-PIVOT_MANUAL_POWER);
    m_manualAlgaeEnabled = true;
  }
  else if(robotcontainer.m_topDriver.GetRightY() < -ALGAE_DEADBAND_CONSTANT)
  {
    robotcontainer.m_claw.SetPower(PIVOT_MANUAL_POWER);
    m_manualAlgaeEnabled = true;
  }
  else if(m_manualAlgaeEnabled)
  {
    robotcontainer.m_claw.SetPower(0);
    m_manualAlgaeEnabled = false;
  }
}

void CmdAlgaeManualPower::End(bool interrupted) 
{
  std::cout << "End Algae Manual Power" << std::endl;
}

bool CmdAlgaeManualPower::IsFinished() 
{
  return false;
}
