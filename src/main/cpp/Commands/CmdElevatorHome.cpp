// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/CmdElevatorHome.h"
#include "constants/Constants.h"
#include "Robot.h"
#include <iostream>

CmdElevatorHome::CmdElevatorHome() 
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdElevatorHome::Initialize() {currentState = ElevatorHomingState::GoHome;}

// Called repeatedly when this Command is scheduled to run
void CmdElevatorHome::Execute() 
{
  switch(currentState)
  {

    case ElevatorHomingState::GoHome:

      std::cout<< "Elevator Go Home"<<std::endl;
      robotcontainer.m_elevator.SetTargetPosition(0);
      currentState = ElevatorHomingState::CheckSensorAtHome;

      break;


    case ElevatorHomingState::CheckSensorAtHome:

      std::cout<< "Elevator CheckSensorAtHome"<<std::endl;
      if(robotcontainer.m_elevator.GetHomeSensorStatus())
      {
        currentState = ElevatorHomingState::MoveDownAfterHome;
      }
      else
      {
        currentState = ElevatorHomingState::MoveUpAfterHome;
      }

      break;

    case ElevatorHomingState::MoveUpAfterHome:

      std::cout<< "Elevator MoveUpAfterHome"<<std::endl;
      robotcontainer.m_elevator.SetElevatorPower(ELEV_HOMING_HIGH_SPEED);
      currentState = ElevatorHomingState::CheckSensorAtHome;

      break;

    case ElevatorHomingState::MoveDownAfterHome:

      std::cout<< "Elevator MoveDownAfterHome"<<std::endl;
      robotcontainer.m_elevator.SetElevatorPower(-ELEV_HOMING_HIGH_SPEED);
      currentState = ElevatorHomingState::CheckSensorForHomingOff;

    break;

    case ElevatorHomingState::CheckSensorForHomingOff:

      std::cout<< "Elevator CheckSensorForHomingOff"<<std::endl;

      if(!robotcontainer.m_elevator.GetHomeSensorStatus())
      {
        currentState = ElevatorHomingState::SetCreep;
      }

      break;

    case ElevatorHomingState::SetCreep:

      std::cout<< "Elevator SetCreep"<<std::endl;
      robotcontainer.m_elevator.SetElevatorPower(-ELEV_HOMING_CREEP_SPEED);
      currentState = ElevatorHomingState::CheckSensorForHomingOn;

      break;

    case ElevatorHomingState::CheckSensorForHomingOn:

      std::cout<< "Elevator CheckSensorForHomingOn"<<std::endl;
      if(robotcontainer.m_elevator.GetHomeSensorStatus())
      {
        currentState = ElevatorHomingState::StopMotor;
      }

      break;

    case ElevatorHomingState::StopMotor:

      std::cout<< "Elevator StopMotor"<<std::endl;
      robotcontainer.m_elevator.Stop();
      currentState = ElevatorHomingState::ResetEncoder;

      break;

    case ElevatorHomingState::ResetEncoder:

      std::cout<< "Elevator ResetEncoder"<<std::endl;
      robotcontainer.m_elevator.ResetEncoderValue();
      currentState = ElevatorHomingState::Finish;

      break;

    case ElevatorHomingState::Finish:

      //std::cout<< "Elevator Finish"<<std::endl;
      break;


  } 
}

// Called once the command ends or is interrupted.
void CmdElevatorHome::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdElevatorHome::IsFinished() {
  return false;
}
