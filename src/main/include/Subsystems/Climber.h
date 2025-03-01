#pragma once

#include "subsystems/Climber.h"
#include "constants/Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Relay.h>


class Climber : public frc2::SubsystemBase 
{

    public:
    
    Climber();

    void Periodic() override;

    bool IsClimberActivated(void);

    void SetClimbPower(double power);

    double GetClimbPower();

    bool GetClimberBeamBreak();

    void DropRamp();

    void ResetRamp();

    void OffRamp();

    private:

    ctre::phoenix6::hardware::TalonFX m_climber{CLIMBER_CAN_ID}; //Constant 

    frc::DigitalInput                 m_climberBeamBreak{CLIMBER_BEAM_BREAK_ID};

    bool m_isClimberActivated;

    frc::Relay                        m_ramp{0};
};

