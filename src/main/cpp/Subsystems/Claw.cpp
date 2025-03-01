#include "Subsystems/Claw.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace rev::spark;

Claw::Claw()
{
    SparkMaxConfig clawconfig{};

    clawconfig
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
        
    m_claw.Configure(clawconfig,
     SparkMax::ResetMode::kResetSafeParameters,
     SparkMax::PersistMode::kPersistParameters);

    SparkMaxConfig pivotconfig{};
    pivotconfig
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50)
        .OpenLoopRampRate(0.3);
    pivotconfig.encoder
        .PositionConversionFactor(100)
        .VelocityConversionFactor(100);
    pivotconfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(0.5, 0.0, 0.0) //Tuning 
        .OutputRange(-0.09, 0.1);     
    pivotconfig.closedLoop.maxMotion
        .MaxVelocity(100)
        .MaxAcceleration(150)
        .AllowedClosedLoopError(0.1);

    m_pivot.Configure(pivotconfig,
     SparkMax::ResetMode::kResetSafeParameters,
     SparkMax::PersistMode::kPersistParameters);


    frc::SmartDashboard::PutNumber("Pivot Current", GetAlgaeCurrent());
    frc::SmartDashboard::PutNumber("Pivot Temp", GetAlgaeTemp());

}

// This method will be called once per scheduler run
void Claw::Periodic() 
{
    frc::SmartDashboard::PutBoolean("Claw Photo Eye", GetClawPhotoEyeFirst());

    frc::SmartDashboard::PutBoolean("Algae Photo Eye", GetAlgaePhotoEye()); 
    
    frc::SmartDashboard::PutBoolean("Coral", IsCoralReady()); 

    frc::SmartDashboard::PutNumber("Pivot Encoder", GetPosition());
}

// --- CLAW ---

void Claw::SetClawPower(double power)
{
    m_claw.Set(power);
}

void Claw::StopClawPower(double power)
{
    m_claw.Set(0);
}

bool Claw::GetClawPhotoEyeFirst(void)
{
    return m_armPhotoeyeFirst.Get();
}

bool Claw::IsCoralReady()
{
    return isCoralReady;
}

// --- ALGAE INTAKE ---

float Claw::GetAlgaeCurrent()
{
    return m_claw.GetOutputCurrent();
}
bool Claw::GetAlgaePhotoEye()
{
    return m_algaePhotoEye.Get();
}
float Claw::GetAlgaeTemp()
{
    return m_claw.GetMotorTemperature();
}
double Claw::GetAlgaePower()
{
    return m_claw.Get();
}

void Claw::SetAlgaePower(double power)
{
    m_claw.Set(power);
}

// --- ALGAE PIVOT ---

  double Claw::GetPosition(void)
  {
    return m_pivotEncoder.GetPosition();
  }
  void Claw::SetPosition(double position)
  {
    m_pivotPID.SetReference( position, rev::spark::SparkMax::ControlType::kMAXMotionPositionControl );  //MaxMotion
  }
  void Claw::SetPower(double power)
  {
    m_pivot.Set(power);
  }

 void Claw::ZeroEncoder(void)
 {
    m_pivotEncoder.SetPosition(0.0);
 }

 bool Claw::IsBallLoaded() const 
{
  return m_ballLoaded;
}

void Claw::SetBallLoaded(bool loaded)
{
    m_ballLoaded = loaded;
}