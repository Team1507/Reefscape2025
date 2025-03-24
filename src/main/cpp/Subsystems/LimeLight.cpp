#include "Subsystems/LimeLight.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>
#include <cmath>
#include "Robot.h"
#include "str/swerve/SwerveDrive.h"
#include "str/DriverstationUtils.h"
#include <frc/Timer.h>

// Helper function to set a double array in NetworkTables.
// This function is used by the inline SetRobotOrientation above.
void setLimelightNTDoubleArray(const std::string& limelightName, const std::string& key, const std::vector<double>& values)
{
    nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->PutNumberArray(key, values);
}

LimeLight::LimeLight(std::string llname) : m_LLName(llname)
{
}

// This method will be called once per scheduler run
void LimeLight::Periodic() 
{
    RunLimeLight();
     auto ntTable = nt::NetworkTableInstance::GetDefault().GetTable(m_LLName);
    
    // Get MegaTag data
    auto botpose = ntTable->GetNumberArray("botpose_wpiblue", {});
    if(botpose.size() >= 6) {
        frc::Pose2d visionPose = toPose2D(botpose);
        
        if(str::IsOnRed()) {
            visionPose = pathplanner::FlippingUtil::flipFieldPose(visionPose);
        }
        
        // Get timestamp from LL latency
        const double latency = ntTable->GetNumber("tl", 0) / 1000.0;
        const units::second_t timestamp = frc::Timer::GetFPGATimestamp() - latency * 1_s;
        
        // Add to pose estimator
        Eigen::Vector3d stdDevs{0.5, 0.5, 10.0}; // Tune based on distance
        swerveDrive.AddVisionMeasurement(visionPose, timestamp, stdDevs);
    }
}

int LimeLight::GetTargetId(void)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("tid", 0);   
}

bool LimeLight::IsTargetValid(void)
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
    // Get vertical target offset angle from NetworkTables.
    double targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("ty", 0);

    // Limelight mounting parameters (adjust as needed)
    double limelightMountAngleDegrees = 10.0; 
    double limelightLensHeightInches = 8.5; 
    double goalHeightInches = 12; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (M_PI / 180.0);

    // Calculate the distance from the limelight to the target.
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
}

void LimeLight::SetPipeline(int value)
{
    nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->PutNumber("pipeline", value);
    std::cout << "SetPipeline= " << value << std::endl;
}

int LimeLight::GetPipeline(void)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(m_LLName)->GetNumber("getpipe", 0); 
}

void LimeLight::SetLastSeenID(void) 
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
        return std::clamp(pidoutput, -1.0, 1.0);
    }
    return 0;
}

double LimeLight::GetTargetYaw(void)
{
    switch(m_lastSeenID) {
        case 18:
        case  7:
            return 0.0; //0
        case 17:
        case  8:
            return 60.0; //60
        case 22:
        case  9:
            return 120.0; //125
        case 21:
        case 10:
            return 180.0; //-174
        case 20:
        case 11:
            return 240.0; //-116
        case 19:
        case  6:
            return 300.0; //-55
        default:
            return -1.0;
    }
}

void LimeLight::RunLimeLight(void)
{
    SetLastSeenID();
    // Update status on the SmartDashboard.
    frc::SmartDashboard::PutBoolean(m_LLName + " Valid", IsTargetValid());
    frc::SmartDashboard::PutNumber(m_LLName + " TID", GetTargetId());
    frc::SmartDashboard::PutNumber(m_LLName + " XAngle", GetTargetHAngle());
    frc::SmartDashboard::PutNumber(m_LLName + " YAngle", GetTargetVAngle());
    frc::SmartDashboard::PutNumber(m_LLName + " Range", GetTargetDistance());
    frc::SmartDashboard::PutNumber(m_LLName + " Last ID", m_lastSeenID);
    frc::SmartDashboard::PutNumber(m_LLName + " Target Yaw", GetTargetYaw());
    // Uncomment the next line if you want to display the PID output.
    // frc::SmartDashboard::PutNumber(m_LLName + " Track PID Out", TrackBranch());
}
