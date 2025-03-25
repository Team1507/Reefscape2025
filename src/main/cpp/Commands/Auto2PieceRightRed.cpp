#include "Commands/Auto2PieceRightRed.h"
#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "Commands/CmdClawAuto.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "Commands/CmdDriveClearAll.h"

Auto2PieceRightRed::Auto2PieceRightRed() 
{
  AddCommands(
    CmdPrintText("Auto 2 Right Red"),
    CmdDriveClearAll(),

    //Go to reef
    CmdDriveToPoint(0.5_m, -0.5_m, 0_deg, 1.5_mps, false, 5_s),
    CmdDriveToPoint(1.5_m, -1.5_m, 0_deg, 3_mps, false, 5_s),
    CmdElevatorToPosition(2),
    CmdDriveToPoint(2.2_m, -2.2_m, -60_deg, 1.5_mps, false, 5_s), //was 2.4, -2.3

    //Score coral
    CmdElevatorToPosition(3),
    CmdDriveToPoint(2.45_m, -2.48_m, -60_deg, 1_mps, true, 3_s), //was 2.48, -2.47 //then 2.28, -2.57 //then 2.38, -2.52 //then 2.43, -2.5 //then 2.46, -2.48
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.65),

    //Get coral at coral station
    CmdDriveToPoint(3.7_m, -1.4_m, -60_deg, 3.5_mps, false, 5_s),
    CmdDriveToPoint(6_m, -0.8_m, -125_deg, 3_mps, false, 5_s),
    frc2::ParallelCommandGroup(   
    CmdDriveToPoint(6.6_m, -0.35_m, -127_deg, 1.25_mps, true, 2_s),
    CmdClawActivate(-1.0)
    ),
 

    //Go to reef
    frc2::ParallelCommandGroup(
    CmdClawActivate(-1.0),
    CmdDriveToPoint(6_m, -0.6_m, -120_deg, 1.5_mps, false, 3_s)
    ),
    CmdDriveToPoint(4.8_m, -1.8_m, -120_deg, 3_mps, false, 3_s),
    frc2::ParallelCommandGroup(
    //CmdClawActivate(-1.0),
    CmdElevatorToPosition(4)
    ),
    CmdDriveToPoint(4.1_m, -2.25_m, -120_deg, 1.5_mps, false, 3_s),

    //Score coral
    CmdElevatorToPosition(3),
    //CmdDriveToPoint(3.9_m, -2.48_m, -120_deg, 1_mps, true, 2_s), //prevois move miised left
    CmdDriveToPoint(3.85_m, -2.44_m, -120_deg, 1_mps, true, 2_s), //work gud on our feild 3/25
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.65),

    //Align to algae
    CmdDriveToPoint(3.9_m, -2.1_m, -120_deg, 1_mps, true, 2_s),
    CmdElevatorToPosition(5),
    CmdDriveClearAll(),

    CmdPrintText("Auto 2 Right Red Done")
  );
}
