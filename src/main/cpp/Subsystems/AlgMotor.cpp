#include "Subsystems/AlgMotor.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace rev::spark;

AlgMotor::AlgMotor() 
{
  SparkMaxConfig Algconfig{};
    Algconfig
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(18);
        
    m_algMotor1.Configure(Algconfig,
     SparkMax::ResetMode::kResetSafeParameters,
     SparkMax::PersistMode::kPersistParameters);
}

// This method will be called once per scheduler run
void AlgMotor::Periodic() 
{
    frc::SmartDashboard::PutBoolean("Algae Photo Eye", GetAlgaePhotoEye()); 
}

void AlgMotor::SetIntakePower(double power)
{
    m_algMotor1.Set(power);
}
bool AlgMotor::GetAlgaePhotoEye()
{
    return m_algaePhotoEye.Get();
}