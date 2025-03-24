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

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
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
    CmdElevatorToPosition(5),
    CmdWait(0.65),

    //Align to Algae
    CmdDriveToPoint(1.2_m, 0.15_m, 0_deg, 1.5_mps, false, 3_s),
    CmdPivotToPos(2),
    CmdWait(0.65),
    frc2::ParallelCommandGroup(
    CmdAlgaeIntake(-1.0),
    CmdDriveToPoint(1.4_m, 0.15_m, 0_deg, 1_mps, true, 2_s)),
    CmdWait(1.0),
    CmdDriveToPoint(1.2_m, 0.15_m, 0_deg, 1_mps, true, 3_s),



    CmdPrintText("Auto 1 Middle End")
  );
}
