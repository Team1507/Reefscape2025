#include "Subsystems/Climber.h"
#include "constants/Constants.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
Climber::Climber()
{
    
    m_isClimberActivated = false;
     m_climber.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

    SparkMaxConfig climberconfig{};

    climberconfig
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
        
    m_climberSpark.Configure(climberconfig,
     SparkMax::ResetMode::kResetSafeParameters,
     SparkMax::PersistMode::kPersistParameters);

}

void Climber::Periodic()
{
    frc::SmartDashboard::PutBoolean("Climber Activated Status", IsClimberActivated());
    frc::SmartDashboard::PutBoolean("Climber Done", GetClimberBeamBreak());
    frc::SmartDashboard::PutBoolean("Climber Beam Break", GetClimberBeamBreak());
    frc::SmartDashboard::PutNumber("Climber Current", GetClimberCurrent().value());
}

void Climber::SetClimbPower(double power, double sparkPower)
{
    m_climber.Set(power);
    m_climberSpark.Set(sparkPower);
}

units::ampere_t Climber::GetClimberCurrent()
{
    auto current = m_climber.GetTorqueCurrent().GetValue();
    return current;
}

double Climber::GetClimbPower(void)
{
    return m_climber.Get();
}

bool Climber::IsClimberActivated(void)
{
    return m_isClimberActivated;
}

bool Climber::GetClimberBeamBreak()
{
    return m_climberBeamBreak.Get();
}

void Climber::DropRamp()
{
    m_ramp.Set(frc::Relay::kReverse);
}

void Climber::ResetRamp()
{
    m_ramp.Set(frc::Relay::kForward);
}

void Climber::OffRamp()
{
    m_ramp.Set(frc::Relay::kOff);
}


