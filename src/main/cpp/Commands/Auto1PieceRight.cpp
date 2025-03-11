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


Auto1PieceRight::Auto1PieceRight() 
{
  
  AddCommands(
    CmdPrintText("Auto 1 Right"),
    CmdDriveClearAll(),

    //Go to reef
    CmdDriveToPoint(0.5_m, -0.5_m, 0_deg, 1.5_mps, false, 5_s),
    CmdDriveToPoint(1.5_m, -1.5_m, 0_deg, 3.5_mps, false, 5_s),
    CmdElevatorToPosition(4),
    CmdDriveToPoint(2.4_m, -2.3_m, -60_deg, 2_mps, false, 5_s),

    //Score coral
    CmdElevatorToPosition(3),
    CmdDriveToPoint(2.48_m, -2.47_m, -60_deg, 1_mps, true, 3_s),
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),

    CmdPrintText("Auto 1 Right Done")
  );

}
