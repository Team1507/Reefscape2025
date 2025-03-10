// #pragma once

// #include <frc2/command/Command.h>
// #include <frc2/command/CommandHelper.h>
// #include "subsystems/Drive.h"
// #include "Robot.h"

// class CmdSmartAlignToReef
//     : public frc2::CommandHelper<frc2::Command, CmdSmartAlignToReef> {
//  public:
//   CmdSmartAlignToReef(bool alignLeft, units::meter_t customOffset = 0_m);

//   void Initialize() override;
//   void Execute() override;
//   void End(bool interrupted) override;
//   bool IsFinished() override;

//  private:
//   bool m_alignLeft;
//   units::meter_t m_customOffset;
//   frc::Pose2d m_targetPose;
//   frc::Timer m_timer;
//   units::meter_t m_lastDistance;
  
//   frc::Pose2d CalculateTargetPose() const;
//   frc::Translation2d GetOptimalApproachVector() const;
// };