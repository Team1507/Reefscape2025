#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include "units/length.h"
#include "units/angle.h"

/**
 * CmdAlignToAprilTag uses your vision system to locate an AprilTag,
 * then computes a target pose with a hard-coded lateral offset
 * (different for left vs right alignment) and a heading that makes
 * the robot face the tag. It then drives to that pose.
 */
class CmdAlignToAprilTag : public frc2::CommandHelper<frc2::Command, CmdAlignToAprilTag> {
 public:
  /**
   * @param alignLeft Pass true for left-side alignment, false for right-side.
   */
  explicit CmdAlignToAprilTag(bool alignLeft);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  bool m_alignLeft;
  frc::Timer m_timer;
  bool m_targetAcquired = false;
  frc::Pose2d m_targetPose;
};
