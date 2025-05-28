#include "Subsystems/Pivot.h"
#include "Constants/Constants.h"
#include "constants/Presets.h"
#include <iostream>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>


Pivot::Pivot() 
{

  // SparkMaxConfig Algconfig{};
  // SparkMaxConfig AlgFollowerConfig{};

  //   Algconfig
  //       .Inverted(false)
  //       .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
  //       .SmartCurrentLimit(18);

  //   AlgFollowerConfig.Apply(Algconfig).Follow(m_algMotor1, true);
        
  //   m_algMotor1.Configure(Algconfig,
  //    SparkMax::ResetMode::kResetSafeParameters,
  //    SparkMax::PersistMode::kPersistParameters);

    // m_algMotor2.Configure(AlgFollowerConfig,
    //  SparkMax::ResetMode::kResetSafeParameters,
    //  SparkMax::PersistMode::kPersistParameters);

  

    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
   // ctre::phoenix6::configs::CANcoderConfiguration canCoderCfg{};
  
    //Encocder Config
 // canCoderCfg.MagnetSensor.MagnetOffset = 0.0_tr;  // adjust if needed to zero the sensor
  //canCoderCfg.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;



  // cfg.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
   // cfg.Feedback.FeedbackRemoteSensorID = m_pivotCancoder.GetDeviceID(); // CANCoder ID for the pivot motor

   m_pivotMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  /* Configure gear ratio */
  ctre::phoenix6::configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

  /* Configure Motion Magic */
  ctre::phoenix6::configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity = 5_tps; // 5 (mechanism) rotations per second cruise
  mm.MotionMagicAcceleration = 7.5_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel 
  mm.MotionMagicJerk = 100_tr_per_s_cu;


  //READ THIS COMMENT: This is the same config as the elevator, WILL NEED TO BE TUNED
  ctre::phoenix6::configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kS = 0.4; // Add 0.25 V output to overcome static friction
  slot0.kV = 0.75; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 45.0; // A position error of 0.2 rotations results in 12 V output
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

  m_pivotMotor.SetControl(m_mmPivot.WithPosition(targetPosition));

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
