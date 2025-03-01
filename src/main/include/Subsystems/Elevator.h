
#pragma once

#include <optional>

#include <frc/TimedRobot.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <wpimath/MathShared.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/DigitalInput.h>


#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>


#include "constants/Constants.h"

#include <iostream>
#include <map>
#include <string>


class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  void Periodic() override;
  void ElevatorSimulationInit();
  void ElevatorSimulationPeriodic();

  float GetElevatorCurrent();
  double GetElevatorTemp();
  double GetElevatorPower();
  double GetElevatorPosition();
  bool GetHomeSensorStatus();

  units::meter_t GetHeight();


  void SetElevatorPower(double power);
  void SetElevatorPosition(float position);

  void SetElevatorCoast();
  void SetElvevatorBrake();

  void ResetEncoderValue();

  bool AtSetpoint();

  void SetTargetPosition(int position);
  void MovePosition(std::string positionName);
  void CheckAtGoal(void);
  void Stop(void);
  void Homing(void);

  units::meter_t ConvertRadiansToHeight(units::radian_t rots);

  bool elevatorL2;
  bool elevatorL3;
  bool elevatorL4;

  bool elevatorLoad;

  bool elevatorLowAlgae;
  bool elevatorHighAlgae;

 private:

 ctre::phoenix6::hardware::TalonFX m_elevatorMotor{ELEVATOR_CAN_ID}; 
 ctre::phoenix6::controls::MotionMagicVoltage m_mmElevator{0_tr};


  ctre::phoenix6::StatusSignal<units::turn_t> PositionSig =
      m_elevatorMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> VelocitySig =
      m_elevatorMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> VoltageSig =
      m_elevatorMotor.GetMotorVoltage();


  int m_printCount = 0;
  
  // units::meter_t elevTolerance = 0.01_m;

  units::turn_t targetPosition = 0_tr;
  units::turn_t temp_High = 0_tr;
  units::turn_t temp_Low = 0_tr;
  bool homing;
  bool isHomed;

  frc::DigitalInput         m_elevatorHomeSensor  {ELEV_HOME_SENSOR};

};
