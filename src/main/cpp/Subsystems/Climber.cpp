#include "Subsystems/Climber.h"
#include "constants/Constants.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber()
{
    
    m_isClimberActivated = false;
     m_climber.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void Climber::Periodic()
{
    frc::SmartDashboard::PutBoolean("Climber Activated Status", IsClimberActivated());
    frc::SmartDashboard::PutBoolean("Climber Done", GetClimberBeamBreak());
    frc::SmartDashboard::PutBoolean("Climber Beam Break", GetClimberBeamBreak());
    frc::SmartDashboard::PutNumber("Climber Current", GetClimberCurrent().value());
}

void Climber::SetClimbPower(double power)
{
    m_climber.Set(power);
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