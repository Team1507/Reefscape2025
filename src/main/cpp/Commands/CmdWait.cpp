#include "commands/CmdWait.h"
#include "Robot.h"
#include <iostream>

CmdWait::CmdWait(double wait) 
{
  m_wait = wait;
}


void CmdWait::Initialize() 
{
  std::cout <<"Shooter Is Waiting... Any Day Now" << std::endl;
  m_timer.Reset();
  m_timer.Start();
}

void CmdWait::Execute() 
{

}

void CmdWait::End(bool interrupted) 
{
  std::cout <<"Shooter Is Done Waiting :)" <<std::endl;
  m_timer.Stop();
}

bool CmdWait::IsFinished()
{
 const units::second_t timeout = units::second_t(m_wait); 
 return m_timer.Get() >= timeout;
}