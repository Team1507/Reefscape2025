#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "Commands/CmdClawAuto.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "Commands/CmdDriveClearAll.h"
#include "Commands/Auto1PieceRight.h"

#include "Commands/Auto1PieceMiddle.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Auto1PieceMiddle::Auto1PieceMiddle() {
  AddCommands(
    CmdPrintText("Auto 1 Middle"),
    CmdDriveClearAll(),

    CmdDriveToPoint(1.6_m, 0_m, 0_deg, 1.5_mps, true, 3_s),

    CmdPrintText("Auto 1 Middle End")
  );
}
