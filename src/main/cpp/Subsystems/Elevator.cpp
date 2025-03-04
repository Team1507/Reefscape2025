#include "subsystems/Elevator.h"
#include <iostream>
#include "sim/PhysicsSim.h"
#include "sim/TalonFXSimProfile.h"

#include <frc/smartdashboard/SmartDashboard.h>


Elevator::Elevator() 
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

  ctre::phoenix6::configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kS = 0.4; // Add 0.25 V output to overcome static friction
  slot0.kV = 5.0; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 60.0; // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 0.0; // No output for integrated error
  slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_elevatorMotor.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }



}
void Elevator::ElevatorSimulationInit() 
{
  PhysicsSim::GetInstance().AddTalonFX(m_elevatorMotor, 0.001_kg_sq_m);
}
void Elevator::ElevatorSimulationPeriodic() 
{
  PhysicsSim::GetInstance().Run();
}

void Elevator::Periodic() 
{
  frc::SmartDashboard::PutNumber("Elevator position", GetElevatorPosition());
  frc::SmartDashboard::PutNumber("Elevator Height", GetHeight().value());
  frc::SmartDashboard::PutNumber("Elevator Temp", GetElevatorTemp());

  frc::SmartDashboard::PutBoolean("Elevator L2", elevatorL2);
  frc::SmartDashboard::PutBoolean("Elevator L3", elevatorL3);
  frc::SmartDashboard::PutBoolean("Elevator L4", elevatorL4);
  frc::SmartDashboard::PutBoolean("Elevator Load", elevatorHome);
  frc::SmartDashboard::PutBoolean("Elevator Low Algae", elevatorLowAlgae);
  frc::SmartDashboard::PutBoolean("Elevator High Algae", elevatorHighAlgae);


}


units::meter_t Elevator::GetHeight() {
  units::turn_t latencyComp =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          PositionSig, VelocitySig);

  units::meter_t avgHeight =
      ConvertRadiansToHeight((latencyComp) / 2.0) * 2;

    return avgHeight;
}

double Elevator::GetElevatorTemp()
{
  return m_elevatorMotor.GetDeviceTemp().GetValue().value();
}

double Elevator::GetElevatorPower() 
{
  return m_elevatorMotor.Get();
}

void Elevator::SetElevatorPower(double power)
{
  m_elevatorMotor.Set(power);
}

void Elevator::SetElevatorPosition(float position)
{
  //m_elevatorMotor.SetControl(m_mmElevator);
}

void Elevator::SetElevatorCoast()
{
  m_elevatorMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Elevator::SetElvevatorBrake()
{
  m_elevatorMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

double Elevator::GetElevatorPosition()
{
  return m_elevatorMotor.GetPosition().GetValueAsDouble();
}

bool Elevator::GetHomeSensorStatus()
{
  return m_elevatorHomeSensor.Get();
}

void Elevator::Stop()
{
  m_elevatorMotor.StopMotor();
}


units::meter_t Elevator::ConvertRadiansToHeight(units::radian_t rots)
{
  return (rots / 1_rad) * (ELEV_PULLY_DIAM / 2);
}

void Elevator::SetTargetPosition (int position)
{

    elevatorL1 = elevatorL2 = elevatorL3 = elevatorL4 = elevatorHome = elevatorLowAlgae = elevatorHighAlgae = false;

  if (position == ELEV_POS_HOME)
  {
    //move elevator home
    targetPosition = ELEV_POSITION_HOME;
    elevatorHome = true;
  }
  else if (position == ELEV_POS_L1)
  {
    targetPosition = ELEV_POSITION_L1;
    elevatorL1 = true;
  }
  else if (position == ELEV_POS_L2)
  {
    targetPosition = ELEV_POSITION_L2;
    elevatorL2 = true;
  }
  else if (position == ELEV_POS_L3)
  {
    targetPosition = ELEV_POSITION_L3;
    elevatorL3 = true;
  }
  else if (position == ELEV_POS_L4)
  {
    targetPosition = ELEV_POSITION_L4;
    elevatorL4 = true;
  }
  else if (position == ELEV_POS_ALG_LOW)
  {
    targetPosition = ELEV_POSITION_ALG_L2;
    elevatorLowAlgae = true;
  }
  else if(position == ELEV_POS_ALG_HIGH)
  {
    targetPosition = ELEV_POSITION_ALG_L3;
    elevatorHighAlgae = true;
  }

  
  m_elevatorMotor.SetControl(m_mmElevator.WithPosition(targetPosition));

  // m_elevatorMotor.SetVoltage
  // (
  //   units::volt_t{
  //     pidController.Calculate(units::meter_t{m_elevatorEncoder.GetDistance()})} +
  //   m_feedforward.Calculate(pidController.GetSetpoint().velocity));
    
}

bool Elevator::AtSetpoint()
{
  temp_High = targetPosition + ELEV_TOLERANCE;
  temp_Low = targetPosition - ELEV_TOLERANCE;

  if (temp_Low < m_elevatorMotor.GetPosition().GetValue() && m_elevatorMotor.GetPosition().GetValue() > temp_High)
  {
    return true;
  } else { return false; }
}

void Elevator::ResetEncoderValue()
{
  m_elevatorMotor.SetPosition(0_tr);
}
