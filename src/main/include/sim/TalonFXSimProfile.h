#pragma once

#include "sim/SimProfile.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/simulation/DCMotorSim.h>

class TalonFXSimProfile : public SimProfile {
    static constexpr units::ohm_t kMotorResistance = 2_mOhm; // Assume 2mOhm resistance for voltage drop calculation
    frc::sim::DCMotorSim _motorSim;
    ctre::phoenix6::sim::TalonFXSimState& _talonFXSim;


public:
    
    TalonFXSimProfile(ctre::phoenix6::hardware::TalonFX& talonFX, units::kilogram_square_meter_t rotorInertia);

    
    void Run();
};