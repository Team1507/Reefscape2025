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
#include "Commands/CmdAlgaeIntake.h"
#include "Commands/CmdAlgaeOuttake.h"
#include "Commands/CmdPivotToPos.h"

#include "Commands/Auto1PieceMiddle.h"


Auto1PieceMiddle::Auto1PieceMiddle() {
  AddCommands(
    CmdPrintText("Auto 1 Middle"),
    CmdDriveClearAll(),

    CmdDriveToPoint(1.3_m, 0_m, 0_deg, 1.5_mps, false, 3_s),
    CmdElevatorToPosition(3),
    CmdWait(0.65),
    CmdDriveToPoint(1.5_m, 0_m, 0_deg, 1_mps, true, 2_s),
    CmdWait(0.65),
    CmdClawOuttake(-1.0),

    CmdPrintText("Auto 1 Middle End")
  );
}
