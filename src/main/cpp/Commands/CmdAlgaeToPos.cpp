#include "Commands/CmdAlgaeToPos.h"
#include "Robot.h"

CmdAlgaeToPos::CmdAlgaeToPos(double position) {m_position = position;}
  
void CmdAlgaeToPos::Initialize()  
{
  std::cout << "CmdElevatorSetPosition " << m_position << std::endl;
  robotcontainer.m_claw.SetPosition( m_position );
  robotcontainer.m_claw.SetPower(0.3);
}

void CmdAlgaeToPos::Execute() {}

void CmdAlgaeToPos::End(bool interrupted) {}

bool CmdAlgaeToPos::IsFinished() 
{
  if (robotcontainer.m_claw.GetPosition() >= m_position - 0.05 && robotcontainer.m_claw.GetPosition() <= m_position + 0.05)
  {
    robotcontainer.m_claw.SetPower(0);
    return true;
  }
  return false;
}
