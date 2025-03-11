#include <iostream>
#include "commands/CmdClawAuto.h"
#include "Robot.h"  
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>  

CmdClawAuto::CmdClawAuto(double power)
    : m_power(power) {
  AddRequirements(&robotcontainer.m_claw);
}

void CmdClawAuto::Initialize() {
  std::cout << "Initialize CmdClawAuto" << std::endl;

  // Force the command to always run in Intake mode
  m_operationMode = OperationMode::Intake;
  currentState = ClawAutoState::RunClawFull;
  std::cout << "CmdClawAuto: Mode set to Intake" << std::endl;
}

void CmdClawAuto::Execute() {
  switch (currentState) {
    case ClawAutoState::RunClawFull:
      robotcontainer.m_claw.SetClawPower(m_power);
      std::cout << "claw Run Full" << std::endl;
      currentState = ClawAutoState::Sensor1;
      break;
    case ClawAutoState::Sensor1:
      if (robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
        std::cout << "claw See coral" << std::endl;
        currentState = ClawAutoState::RunClawCreep;
      }
      break;
    case ClawAutoState::RunClawCreep:
      robotcontainer.m_claw.SetClawPower(-0.5);
      std::cout << "claw Creep" << std::endl;
      currentState = ClawAutoState::NotDetected;
      break;
    case ClawAutoState::NotDetected:
      if (!robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
        std::cout << "claw no see" << std::endl;
        currentState = ClawAutoState::StopMotor;
      }
      break;
    case ClawAutoState::StopMotor:
      robotcontainer.m_claw.StopClawPower(0);
      robotcontainer.m_claw.isCoralReady = true;
      currentState = ClawAutoState::EndState;
      break;
    case ClawAutoState::EndState:
      break;
    default:
      break;
  }
}

void CmdClawAuto::End(bool interrupted) {
  std::cout << "End CmdClawAuto" << std::endl;
  robotcontainer.m_claw.StopClawPower(0);
}

bool CmdClawAuto::IsFinished() {
  return currentState == ClawAutoState::EndState;
}