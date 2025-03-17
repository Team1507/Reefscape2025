// CmdSmartAlignToReef.h
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include "Robot.h"

class CmdSmartAlignToReef
    : public frc2::CommandHelper<frc2::Command, CmdSmartAlignToReef> {
 public:
  CmdSmartAlignToReef(bool alignLeft,  // true = align left, false = right
                     units::meters_per_second_t velocity, 
                     units::second_t timeout);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  bool m_alignLeft;
  units::meters_per_second_t m_velocity;
  units::second_t m_timeout;
  frc::Timer m_timer;
  
  frc::Pose2d m_targetPose;
  units::meter_t lOffset{0};
  units::meter_t rOffset{0};
};