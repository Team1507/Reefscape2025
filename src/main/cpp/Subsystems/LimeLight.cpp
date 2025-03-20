#include "subsystems/LimeLight.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>

#define PI 3.14159265358979323846


LimeLight::LimeLight(std::string llname) 
{
    m_LLName = llname;
}

// This method will be called once per scheduler run
void LimeLight::Periodic() 
{
    RunLimeLight();
}

int    LimeLight::GetTargetId(void)
{
     return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("tid", 0);   
}
bool   LimeLight::IsTargetValid(void)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("tv", 0);   
}

double LimeLight::GetTargetHAngle(void)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("tx", 0);
}

double LimeLight::GetTargetVAngle(void)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("ty", 0);   
}
double LimeLight::GetTargetDistance(void)
{
  // calibrate these values 
  double h1 = frc::SmartDashboard::GetNumber("LL_h1", 35.5);   // Height of Limelight 
  double h2 = frc::SmartDashboard::GetNumber("LL_h2", 103.50);  // Height of target
  double a1 = frc::SmartDashboard::GetNumber("LL_a1", 32.0);    // Limelight's mounting angle (degrees)

  if (!IsTargetValid()) {
    return 0.0;
  }

  // Get the vertical offset from the target (degrees)
  double a2 = nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("ty", 0.0);

  // Total angle in degrees and conversion to radians
  double totalAngleDeg = a1 + a2;
  constexpr double DEG2RAD = PI / 180.0;
  double totalAngleRad = totalAngleDeg * DEG2RAD;

  // Ensure the denominator is not too small (to avoid division by zero)
  double tanAngle = std::tan(totalAngleRad);
  if (std::abs(tanAngle) < 1e-6) {
    std::cout << "Warning: tan(totalAngle) is nearly zero. Returning 0.0 for distance." << std::endl;
    return 0.0;
  }

  double distance = (h2 - h1) / tanAngle;

  // Check for negative distance which indicates a potential calibration error
  if (distance < 0) {
    std::cout << "Warning: computed distance (" << distance 
              << ") is negative. Check calibration values." << std::endl;
    return 0.0;
  }

  return distance;
}
void   LimeLight::SetPipeline(int value)
{
    nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->PutNumber("pipeline", value);
    std::cout <<"SetPipeline= " << value << std::endl;
}
int   LimeLight::GetPipeline(void)
{
   return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("getpipe", 0); 
}

void    LimeLight::SetLastSeenID(void) 
{
    if (IsTargetValid() && (1 <= GetTargetId()) && (GetTargetId() <= 22))
    {
        m_lastSeenID = GetTargetId();
    }
    
}

double LimeLight::TrackBranch()
{
    if (IsTargetValid())
    {
        double pidoutput = m_positionBranchPID.Calculate(GetTargetHAngle(), 0);
        return  std::clamp(pidoutput, -1.0, 1.0);
        //GetTargetHAngle();
    }
    return 0;
}

//Function to return the optimal angle for the drivebase to be square to the reef
double  LimeLight::GetTargetYaw(void)
{
    switch(m_lastSeenID) {
        case 18:
        case  7:
            return 0.0;
            break;
        case 17:
        case  8:
            return 60.0;
            break;
        case 22:
        case  9:
            return 120.0;
            break;
        case 21:
        case 10:
            return 180.0;
            break;
        case 20:
        case 11:
            return 240.0;
            break;
        case 19:
        case  6:
            return 300.0;
            break;
        default:
            return -1.0;
    }
}


void    LimeLight::RunLimeLight(void)
{
    SetLastSeenID();
        //Status Update
    frc::SmartDashboard::PutBoolean(m_LLName + " Valid",        IsTargetValid() );
    frc::SmartDashboard::PutNumber (m_LLName + " TID",          GetTargetId() );
    frc::SmartDashboard::PutNumber (m_LLName + " HAngle",       GetTargetHAngle()  );
    frc::SmartDashboard::PutNumber (m_LLName + " YAngle",       GetTargetVAngle()  );
    frc::SmartDashboard::PutNumber (m_LLName + " Range",        GetTargetDistance()  );
    frc::SmartDashboard::PutNumber (m_LLName + " Last ID",      m_lastSeenID  );
    frc::SmartDashboard::PutNumber (m_LLName + " Target Yaw",   GetTargetYaw()  );
  //frc::SmartDashboard::PutNumber (m_LLName + " Track PID Out",TrackBranch()  );
}