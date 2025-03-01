#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Timer.h>


class CmdClimberActivate
    : public frc2::CommandHelper<frc2::Command, CmdClimberActivate> {
 public:

  CmdClimberActivate(double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  enum class stateClimber
  {
    RunClimberFull,
    Sensor1,
    StopMotor,
    EndState,
  };

  stateClimber currentState = stateClimber::RunClimberFull;

  private:

  double m_power;

  frc::Timer m_timer;
};