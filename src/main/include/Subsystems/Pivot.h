// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include "constants/Constants.h"

class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  double GetPosition(void);

  void SetPosition(double position);

  double GetTemperature(void);

  void SetPower(double power);

  double GetPower(void);

  double GetPosition(void);

  void SetTargetPosition(int position);

  bool pivotHome;
  bool pivotOpen;

 private:

 units::turn_t targetPosition = 0.2_tr;

 ctre::phoenix6::hardware::TalonFX m_pivotMotor{PIVOT_FALCON_CAN_ID}; 
 ctre::phoenix6::controls::MotionMagicVoltage m_mmPivot{0_tr};

};
