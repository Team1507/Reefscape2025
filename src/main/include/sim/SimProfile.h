#pragma once

#include <units/time.h>

class SimProfile {
    units::second_t _lastTime;
    bool _running = false;

public:
    virtual void Run() = 0;

protected:
    units::second_t GetPeriod();
};