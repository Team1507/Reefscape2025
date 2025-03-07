#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/time.h>
#include <frc/Timer.h>

class CmdDriveToPoint
    : public frc2::CommandHelper<frc2::Command, CmdDriveToPoint> {
public:
    CmdDriveToPoint(units::meter_t x, units::meter_t y, units::degree_t heading,
                   units::meters_per_second_t velocity, bool stop, units::second_t timeout);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    units::meter_t m_finalX;
    units::meter_t m_finalY;
    units::degree_t m_finalH;
    units::meters_per_second_t m_velocity;
    bool m_stop;
    units::second_t m_timeout;
    
    frc::Timer m_timer;
    units::meter_t m_startDistance;
};