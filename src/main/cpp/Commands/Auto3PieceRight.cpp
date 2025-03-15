#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "Commands/CmdClawAuto.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "Commands/CmdDriveClearAll.h"
#include "Commands/Auto3PieceRight.h"

Auto3PieceRight::Auto3PieceRight() 
{
  AddCommands(
    CmdPrintText("Auto 3 Right"),
    CmdDriveClearAll(),

    //Go to reef
    CmdDriveToPoint(0.5_m, -0.5_m, 0_deg, 1.5_mps, false, 5_s),
    CmdDriveToPoint(1.5_m, -1.5_m, 0_deg, 4.5_mps, false, 5_s),
    CmdElevatorToPosition(4),
    CmdDriveToPoint(2.4_m, -2.3_m, -60_deg, 2.5_mps, false, 5_s),

    //Score coral
    CmdElevatorToPosition(3),
    CmdDriveToPoint(2.48_m, -2.47_m, -60_deg, 1.25_mps, true, 3_s),
    CmdWait(0.35),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.2),

    //Get coral at coral station
    CmdDriveToPoint(3.7_m, -1.4_m, -60_deg, 4.5_mps, false, 5_s),
    CmdDriveToPoint(6_m, -0.8_m, -125_deg, 3.5_mps, false, 5_s),
    frc2::ParallelCommandGroup(
    CmdDriveToPoint(6.6_m, -0.35_m, -127_deg, 1.25_mps, true, 2_s),
    CmdClawActivate(-1.0)
    ),
 

    //Go to reef
    CmdDriveToPoint(6_m, -0.6_m, -120_deg, 2.5_mps, false, 3_s),
    CmdDriveToPoint(4.8_m, -1.8_m, -120_deg, 4_mps, false, 3_s),
    CmdElevatorToPosition(4),
    CmdDriveToPoint(4.1_m, -2.25_m, -120_deg, 2.5_mps, false, 3_s),

    //Score coral
    CmdElevatorToPosition(3),
    CmdDriveToPoint(3.9_m, -2.48_m, -120_deg, 1.5_mps, true, 2_s),
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.2),

    CmdPrintText("Auto 3 Right Done")
  );
}
