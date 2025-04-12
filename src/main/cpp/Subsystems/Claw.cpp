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

}

// This method will be called once per scheduler run
void Claw::Periodic() 
{
    frc::SmartDashboard::PutBoolean("Claw Photo Eye", GetClawPhotoEyeFirst());

   // frc::SmartDashboard::PutBoolean("Algae Photo Eye", GetAlgaePhotoEye()); 
    
    frc::SmartDashboard::PutBoolean("Coral", IsCoralReady()); 

    // frc::SmartDashboard::PutNumber("Pivot Encoder", GetPosition());
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
