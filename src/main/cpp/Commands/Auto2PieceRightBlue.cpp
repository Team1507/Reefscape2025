#include "Commands/Auto2PieceRightBlue.h"
#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "Commands/CmdClawAuto.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "Commands/CmdDriveClearAll.h"
#include "Commands/CmdPivotToPos.h"

Auto2PieceRightBlue::Auto2PieceRightBlue() 
{
  AddCommands(
    CmdPrintText("Auto 2 Right Blue"),
    CmdDriveClearAll(),

    //Go to reef
    CmdPivotToPos(1),
    CmdDriveToPoint(0.5_m, -0.5_m, 0_deg, 1.5_mps, false, 5_s),
    CmdDriveToPoint(1.5_m, -1.5_m, 0_deg, 3_mps, false, 5_s),
    CmdElevatorToPosition(2),
    CmdDriveToPoint(2.2_m, -2.2_m, -60_deg, 1.5_mps, false, 5_s), //was 2.4, -2.3

    //Score coral
    CmdElevatorToPosition(3),
    CmdDriveToPoint(2.45_m, -2.48_m, -60_deg, 1_mps, true, 3_s), //was 2.48, -2.47 //then 2.28, -2.57 //then 2.38, -2.52 //then 2.43, -2.5 //then 2.46, -2.48
    // CmdDriveToPoint(2.43_m, -2.5_m, -60_deg, 1_mps, true, 3_s), //2.40_m, -2.51_m
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.65),

    //Get coral at coral station
    CmdDriveToPoint(3.7_m, -1.4_m, -60_deg, 3.5_mps, false, 5_s),
    CmdDriveToPoint(6_m, -0.8_m, -125_deg, 3_mps, false, 5_s),
    frc2::ParallelCommandGroup(   
    CmdDriveToPoint(6.75_m, -0.25_m, -127_deg, 1.25_mps, true, 2_s),
    CmdClawActivate(-1.0)
    ),
 

    //Go to reef
    frc2::ParallelCommandGroup(
    CmdClawActivate(-1.0),
    CmdDriveToPoint(6_m, -0.6_m, -120_deg, 1.5_mps, false, 3_s)
    ),
   frc2::ParallelCommandGroup( CmdDriveToPoint(4.8_m, -1.8_m, -120_deg, 3_mps, false, 3_s),
    CmdClawActivate(-1.0)),
   // frc2::ParallelCommandGroup(
    // CmdClawActivate(-1.0),
    CmdElevatorToPosition(4),
   // ),
    CmdDriveToPoint(4.1_m, -2.25_m, -120_deg, 1.5_mps, false, 3_s),

    //Score coral
    CmdElevatorToPosition(3),
    //CmdDriveToPoint(3.9_m, -2.48_m, -120_deg, 1_mps, true, 2_s), //prevois move miised left
    //CmdDriveToPoint(3.85_m, -2.44_m, -120_deg, 1_mps, true, 2_s), //work gud on our feild 3/25
    //CmdDriveToPoint(3.83_m, -2.45_m, -120_deg, 1_mps, true, 2_s), //Missed left 2nd peice
    CmdDriveToPoint(3.83_m, -2.45_m, -122_deg, 1_mps, true, 2_s),
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.65),

    //Back to Station
    frc2::ParallelCommandGroup(
      CmdDriveToPoint(6.5_m, -0.35_m, -127_deg, 2_mps, true, 2_s),
      CmdClawActivate(-1.0)),

    //BAck To Reef
    CmdDriveToPoint(3.53_m, -2.3_m, -117_deg, 1_mps, true, 3_s),

    CmdPrintText("Auto 2 Right Done Blue")
  );
}
