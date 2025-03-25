#include <iostream>
#include "commands/CmdClawActivate.h"
#include "Robot.h"  
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>  
#include <frc2/command/button/Trigger.h>

#define CLAW_INCRIMENT 10
int claw_Incriment = 0;

CmdClawActivate::CmdClawActivate(double power)
    : m_power(power) {
  AddRequirements(&robotcontainer.m_claw);
}

void CmdClawActivate::Initialize() {
  std::cout << "Initialize CmdClawActivate" << std::endl;

  // Force the command to always run in Intake mode
  m_operationMode = OperationMode::Intake;
  currentState = ClawState::RunClawFull;
  std::cout << "CmdClawActivate: Mode set to Intake" << std::endl;
}

void CmdClawActivate::Execute() {

  switch (currentState) {
    case ClawState::RunClawFull:
      robotcontainer.m_claw.SetClawPower(m_power);
      std::cout << "claw Run Full" << std::endl;
      currentState = ClawState::Sensor1;

      
      break;
    case ClawState::Sensor1:

      if (robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
         //if(robotcontainer.m_climber.GetClimberBeamBreak() == false)
      //{
        // if(claw_Incriment == CLAW_INCRIMENT)
        // {
          std::cout << "claw See coral" << std::endl;
          currentState = ClawState::RunClawCreep;

          

          
        // }
        // else
        // {
        //   claw_Incriment ++;
        //   std::cout << "Claw Incroment" << std::endl;
        // }
      //}
        
      }
      else if(robotcontainer.m_claw.m_clawStop == true)
          {
            std::cout << "STATE BROKEN" << std::endl;
            robotcontainer.m_claw.m_clawStop = false;
            currentState = ClawState::StopMotor;
          }


      break;
    case ClawState::RunClawCreep:
      robotcontainer.m_claw.SetClawPower(-0.5);
      std::cout << "claw Creep" << std::endl;
      currentState = ClawState::NotDetected;
      break;
    case ClawState::NotDetected:
      if (!robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
        std::cout << "claw no see" << std::endl;
        currentState = ClawState::RunBackwardUntilClawSeen;
      }
      break;
    case ClawState::RunBackwardUntilClawSeen:
      robotcontainer.m_claw.SetClawPower(0.25);
      std::cout << "claw backword" << std::endl;
      if (robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
        currentState = ClawState::StopMotor;
      }
      else if(robotcontainer.m_claw.m_clawStop == true)
          {
            std::cout << "STATE BROKEN" << std::endl;
            robotcontainer.m_claw.m_clawStop = false;
            currentState = ClawState::StopMotor;
          }

      break;
    case ClawState::StopMotor:
      robotcontainer.m_claw.StopClawPower(0);
      robotcontainer.m_claw.isCoralReady = true;
      currentState = ClawState::EndState;
      break;
    case ClawState::EndState:
      break;
    default:
      break;
  }
}

void CmdClawActivate::End(bool interrupted) {
  std::cout << "End CmdClawActivate" << std::endl;
  robotcontainer.m_claw.StopClawPower(0);
}

bool CmdClawActivate::IsFinished() {
  return currentState == ClawState::EndState;
}