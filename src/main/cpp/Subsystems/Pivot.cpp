#include "Subsystems/Pivot.h"
#include "Constants/Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

Pivot::Pivot() 
{
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};

  /* Configure gear ratio */
  ctre::phoenix6::configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

  /* Configure Motion Magic */
  ctre::phoenix6::configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity = 10_tps; // 5 (mechanism) rotations per second cruise
  mm.MotionMagicAcceleration = 15_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel 
  mm.MotionMagicJerk = 100_tr_per_s_cu;


  //READ THIS COMMENT: This is the same config as the elevator, WILL NEED TO BE TUNED
  ctre::phoenix6::configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kS = 0.4; // Add 0.25 V output to overcome static friction
  slot0.kV = 5.0; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 60.0; // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 0.0; // No output for integrated error
  slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

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

  frc::SmartDashboard::PutNumber("Pivot position", GetPosition());
  frc::SmartDashboard::PutNumber("Pivot Temp", GetTemperature());

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

double Pivot::GetPosition()
{
  return m_pivotMotor.GetPosition().GetValue().value();
}

void Pivot::SetTargetPosition(int position)
{
    pivotHome = pivotOpen = false;

    if (position == ALGAE_POS_HOME)
  {
    //move elevator home
    targetPosition = PIVOT_POSITION_HOME;
    pivotHome = true;
  }
  else if (position == ALGAE_POS_OPEN)
  {
    targetPosition = PIVOT_POSITION_OPEN;
    pivotOpen = true;
  }

  m_pivotMotor.SetControl(m_mmPivot.WithPosition(targetPosition));

}