#include <iostream>
#include "commands/CmdClawActivate.h"
#include "Robot.h"  
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>  // Include the timer header

// Ensure you declare this in your header file (CmdClawActivate.h):
// private:
//   frc::Timer m_timer;

CmdClawActivate::CmdClawActivate(double power)
    : m_power(power) {
  AddRequirements(&robotcontainer.m_claw);
}

void CmdClawActivate::Initialize() {
  std::cout << "Initialize CmdClawActivate" << std::endl;
  
  // Determine the mode based on whether a coral piece is already held.
  if (robotcontainer.m_claw.isCoralReady) {
    m_operationMode = OperationMode::ShootOut;
    currentState = ClawState::ShootOutStart;
    std::cout << "CmdClawActivate: Mode set to ShootOut" << std::endl;
  } else {
    m_operationMode = OperationMode::Intake;
    currentState = ClawState::RunClawFull;
    std::cout << "CmdClawActivate: Mode set to Intake" << std::endl;
  }
}

void CmdClawActivate::Execute() {
  if (m_operationMode == OperationMode::Intake) {
    // Intake sequence remains unchanged.
    switch (currentState) {
      case ClawState::RunClawFull:
        robotcontainer.m_claw.SetClawPower(m_power);
        std::cout << "claw Run Full" << std::endl;
        currentState = ClawState::Sensor1;
        break;
      case ClawState::Sensor1:
        if (robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
          std::cout << "claw See coral" << std::endl;
          currentState = ClawState::RunClawCreep;
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
        robotcontainer.m_claw.SetClawPower(0.1);
        std::cout << "claw backword" << std::endl;
        if (robotcontainer.m_claw.GetClawPhotoEyeFirst()) {
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
  } else {  // OperationMode::ShootOut
  const units::second_t shootOutDuration = units::second_t(1.5);
    switch (currentState) {
      case ClawState::ShootOutStart:
        // Start shootout by running the motor and starting the timer.
        robotcontainer.m_claw.SetClawPower(-1.0);
        m_timer.Reset();
        m_timer.Start();
        currentState = ClawState::ShootOutRunning;
        break;
      case ClawState::ShootOutRunning:
        // After 1.5 seconds, transition to stop.
        if (m_timer.Get() >= shootOutDuration) {
          currentState = ClawState::ShootOutStop;
          m_timer.Stop();
        }
        break;
      case ClawState::ShootOutStop:
        robotcontainer.m_claw.StopClawPower(0);
        robotcontainer.m_claw.isCoralReady = false;
        currentState = ClawState::EndState;
        break;
      case ClawState::EndState:
        // Sequence complete.
        break;
      default:
        break;
    }
  }
}

void CmdClawActivate::End(bool interrupted) {
  std::cout << "End CmdClawActivate" << std::endl;
  robotcontainer.m_claw.StopClawPower(0);
}

bool CmdClawActivate::IsFinished() {
  return currentState == ClawState::EndState;
}
