#include "Robot.h"

#include "Commands/CmdElevatorDpad.h"
#include "Constants/Constants.h"
CmdElevatorDpad::CmdElevatorDpad(dPadPosition position) 
{
  m_position = position;
}

void CmdElevatorDpad::Initialize() 
{

  switch( m_position )
  {

    //----------------------------
    case DPAD_UP:
     if (robotcontainer.m_topDriver.A().Get()) {
        std::cout << "DPAD UP A" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_L1);
      } else {
        std::cout << "DPAD UP" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_L4);
      }
      break;
      
    //----------------------------
    case DPAD_DOWN:

      if(robotcontainer.m_topDriver.A().Get() == true)
      {
        std::cout<< "DPAD DOWN A" << std::endl;
         robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_HOME);
  
      }
      else
      {
        std::cout<< "DPAD DOWN" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_HOME);
      }

      break;

    //----------------------------
    case DPAD_LEFT:
      if(robotcontainer.m_topDriver.A().Get() == true)
      {
        std::cout<< "DPAD LEFT A" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_ALG1);

      }
      else
      {
        std::cout<< "DPAD LEFT" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_L3);
      }
      break;

    //----------------------------
    case DPAD_RIGHT:
      if(robotcontainer.m_topDriver.A().Get() == true)
      {
        std::cout<< "DPAD RIGHT A" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_ALG2);
      }
      else
      {
        std::cout<< "DPAD RIGHT" << std::endl;
        robotcontainer.m_elevator.SetElevatorPosition(ELEV_POS_L2);
      }
      break;

    //----------------------------
    default:
       if(robotcontainer.m_topDriver.A().Get() == true)
      {
        std::cout<< "Unknown DPAD Input A" << std::endl;
      }
      else
      {
        std::cout<< "Unknown DPAD Input" << std::endl;
      }
      break;

  }
}

void CmdElevatorDpad::Execute() {}

void CmdElevatorDpad::End(bool interrupted) {}

bool CmdElevatorDpad::IsFinished() 
{
  return true;
}