#include "Subsystems/Pivot.h"
#include "Constants/Constants.h"
#include "constants/Presets.h"
#include <iostream>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>


Pivot::Pivot() {
    // === CANcoder Configuration ===
//    ctre::phoenix6::configs::CANcoderConfiguration canCoderCfg{};

    // Set direction (e.g., CCW positive)
    // canCoderCfg.MagnetSensor.SensorDirection =
    //     ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;

    // // (Optional) Adjust magnet offset if needed to zero position
    // // canCoderCfg.MagnetSensor.MagnetOffset = 0.0_tr;

    // // Apply CANCoder config
    // ctre::phoenix::StatusCode canStatus = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    //     canStatus = m_pivotCancoder.GetConfigurator().Apply(canCoderCfg);
    //     if (canStatus.IsOK()) break;
    // }
    // if (!canStatus.IsOK()) {
    //     std::cout << "Failed to configure CANCoder: " << canStatus.GetName() << std::endl;
    // }

    // === TalonFX Configuration ===
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};

    // Use fused CANCoder as the feedback source
  //  cfg.Feedback.FeedbackSensorSource =
       // ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
   // cfg.Feedback.FeedbackRemoteSensorID = m_pivotCancoder.GetDeviceID();
   // cfg.Feedback.RotorToSensorRatio = 12.8;
    

    // Set the gear ratio: number of motor rotations per mechanism rotation
    cfg.Feedback.SensorToMechanismRatio = 12.8;

    // Set neutral mode to brake
    m_pivotMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    // === Motion Magic Configuration ===
    ctre::phoenix6::configs::MotionMagicConfigs &mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 3_tps;             // 5 rotations per second
    mm.MotionMagicAcceleration = 6.0_tr_per_s_sq;     // ~0.5 sec to max velocity
    mm.MotionMagicJerk = 90_tr_per_s_cu;             // ~0.1 sec to max accel

    // === Slot 0 PID Config ===
    ctre::phoenix6::configs::Slot0Configs &slot0 = cfg.Slot0;
    slot0.kS = 0.4;
    slot0.kV = 0.75;
    slot0.kA = 0.01;
    slot0.kP = 55.0;
    slot0.kI = 0.0;
    slot0.kD = 0.5;

    // Apply TalonFX configuration
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_pivotMotor.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
}



// This method will be called once per scheduler run
void Pivot::Periodic() 
{
  frc::SmartDashboard::PutNumber("Pivot position", GetPivotPosition());
  frc::SmartDashboard::PutNumber("Pivot Temp", GetTemperature());
  frc::SmartDashboard::PutNumber("Encoder Pos", std::round(GetEncoderPosition() * 1000.0) / 1000.0);
}


double Pivot::GetTemperature()
{
  return m_pivotMotor.GetDeviceTemp().GetValue().value();
}

double Pivot::GetPower() 
{
  return m_pivotMotor.Get();
}

void Pivot::SetPower(double power)
{
  m_pivotMotor.Set(power);
}

double Pivot::GetPivotPosition()
{
  return m_pivotMotor.GetPosition().GetValueAsDouble();
}

double Pivot::GetEncoderPosition()
{
  return m_pivotCancoder.GetPosition().GetValueAsDouble();
}


void Pivot::SetTargetPosition(int position)
{
    //double canTurns = m_pivotCancoder.GetPosition().GetValueAsDouble(); 
    pivotHome = pivotOpen = false;

  if (position == ALGAE_POS_HOME)
  {
    //move elevator home
    targetPosition = PIVOT_POSITION_HOME;
  }
  else if (position == ALGAE_POS_SCORE)
  {
    targetPosition = PIVOT_POSITION_SCORE;
    
  }
  else if (position == ALGAE_POS_BARGE)
  {
    targetPosition = PIVOT_BARGE;
    
  }
  else if (position == ALGAE_POS_FLOOR_CORAL)
  {
    targetPosition = PIVOT_FLOOR_CORAL;
    
  }
  else if (position == ALGAE_POS_FLOOR_ALGAE)
  {
    targetPosition = PIVOT_FLOOR_ALGAE;
    
  }
  else if (position == ALGAE_POS_CLOSE_HOME)
  {
    targetPosition = PIVOT_CLOSE_HOME;
  }
  else if (position == ALGAE_POS_PROSESS)
  {
    targetPosition = PIVOT_PROSESS;
  }
  else if (position == ALGAE_POS_INTAKE)
  {
    targetPosition = PIVOT_INTAKE;
  }

 m_pivotMotor.SetControl(m_mmPivot.WithPosition({targetPosition}));

}

void Pivot::SetManualTarget(double targetPosition) {
    m_pivotMotor.SetControl(m_mmPivot.WithPosition(units::turn_t{targetPosition}));
}

void Pivot::SetPivotCoast()
{
  m_pivotMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Pivot::SetPivotBrake()
{
  m_pivotMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void Pivot::ResetEncoderValue()
{
  m_pivotMotor.SetPosition(0_tr);
}

// void Pivot::SetIntakePower(double power)
// {
//     // Set the algae intake motor power
//    m_algMotor1.Set(power);
// }
