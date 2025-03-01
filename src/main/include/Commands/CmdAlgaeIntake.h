#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <units/time.h>

/**
 * @brief Command to either intake algae (if a ball isn’t loaded)
 *        or shoot the ball out (if a ball is already loaded).
 *
 * On the first button press the command runs in intake mode until the photeye is triggered,
 * setting the ball-loaded flag. On the next press (when the ball is loaded), it shoots the ball out.
 */
class CmdAlgaeIntake 
    : public frc2::CommandHelper<frc2::Command, CmdAlgaeIntake> {
 public:
  /**
   * Constructs the command.
   * @param power The motor power used for intake or shooting.
   */
  explicit CmdAlgaeIntake(double power);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  double m_power;
  frc::Timer m_timer;

  enum class Mode { Intake, ShootOut };
  Mode m_mode;
};
