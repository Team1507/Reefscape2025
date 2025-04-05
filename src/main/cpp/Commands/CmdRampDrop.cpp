#include "Commands/CmdRampDrop.h"
#include "iostream"
#include "Robot.h"

CmdRampDrop::CmdRampDrop() 
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdRampDrop::Initialize() 
{
  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer
}

// Called repeatedly when this Command is scheduled to run
void CmdRampDrop::Execute() 
{
  robotcontainer.m_climber.DropRamp();
}

// Called once the command ends or is interrupted.
void CmdRampDrop::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdRampDrop::IsFinished() 
{
  const units::second_t timeout = units::second_t(1.5);
  if(m_timer.Get() >= timeout)
  {
    robotcontainer.m_climber.OffRamp();
    return true;
  }
  else 
  {
    return false;
  }

}