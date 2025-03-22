#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

/**
 * This command uses joystick input to drive the pivot via Motion Magic.
 */
class CmdPivotManual : public frc2::CommandHelper<frc2::Command, CmdPivotManual> {
 public:
  CmdPivotManual();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private:
  bool m_manualPivotEnabled;
};
