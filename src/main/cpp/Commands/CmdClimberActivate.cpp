#include "Commands/CmdClimberActivate.h"
#include "Robot.h"
#include "subsystems/Climber.h"

#include <frc/XboxController.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  


#include <iostream> 

#define CLIMBER_INCRIMENT 1
int incriment = 0;

CmdClimberActivate::CmdClimberActivate(double power, double sparkPower) 
{
  m_power = power;
  m_sparkPower = sparkPower;
  AddRequirements(&robotcontainer.m_climber); //Calls Climber through container
}


void CmdClimberActivate::Initialize() 
{
  std::cout << "Starting CmdClimberActivated" << std::endl; //Lets us know the command has started
  currentState = stateClimber::RunClimberFull;
  // if(m_timer.Get() <= units::second_t(0.0))
  // {
  //   m_timer.Reset(); //Resets the timer
  //   m_timer.Start(); //Starts the timer
  // }
}


void CmdClimberActivate::Execute() 
{
  // const units::second_t timeout = units::second_t(0.3);

  // while (m_timer.Get() <= timeout)
  // {
  //   if(robotcontainer.m_topDriver.X().Get())
  //   {
  //     robotcontainer.m_climber.ReleaseRamp();
  //   }
  //   if(robotcontainer.m_topDriver.B().Get())
  //   {
  //     robotcontainer.m_climber.SetClimbPower(m_power);
  //   }
  // }

  //robotcontainer.m_climber.SetClimbPower(0);

  switch(currentState)
  {
    case stateClimber::RunClimberFull:
      robotcontainer.m_climber.SetClimbPower(m_power, m_sparkPower);
      currentState = stateClimber::Sensor1;
      std::cout << "RunClimberFull" << std::endl;
      break;
    
    case stateClimber::Sensor1:
      std::cout << "Sensor1" << std::endl;
      if(robotcontainer.m_climber.GetClimberBeamBreak() == false)
      {
        if(incriment == CLIMBER_INCRIMENT)
        {
          currentState = stateClimber::StopMotor;
          std::cout << "Sensor Off" << std::endl;
        }
        else
        {
          incriment ++;
          std::cout << "Climber Incroment" << std::endl;
        }
      }
      break;

    case stateClimber::StopMotor:
      robotcontainer.m_climber.SetClimbPower(0,0);
      currentState = stateClimber::EndState;
      std::cout << "StopMotor" << std::endl;
      break;

    case stateClimber::EndState:
      break;
  }

  // if(robotcontainer.m_climber.GetClimberBeamBreak())
  //   {
  //     robotcontainer.m_climber.SetClimbPower(0.2);
  //   }
}


void CmdClimberActivate::End(bool interrupted) 
{
  std::cout << "End CmdMotorSpin" << std::endl; //Lets Us know the command is ending
  
  robotcontainer.m_climber.SetClimbPower(0,0);

  // m_timer.Stop();
  // m_timer.Reset();
}


bool CmdClimberActivate::IsFinished() 
{
  return false;
}