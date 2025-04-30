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
#include "Commands/CmdPivotIntake.h"
#include "Constants/Presets.h"

#include "Commands/Auto1PieceMiddleAlg.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Auto1PieceMiddleAlg::Auto1PieceMiddleAlg() {
  AddCommands(
    CmdPrintText("Auto 1 Alg Middle"),
    CmdDriveClearAll(),

    CmdPivotToPos(1),
    CmdDriveToPoint(1.3_m, 0_m, 0_deg, 1.5_mps, false, 3_s),
    CmdElevatorToPosition(3),
    CmdWait(0.65),
    CmdDriveToPoint(1.5_m, 0_m, 0_deg, 1_mps, true, 2_s),
    CmdWait(0.65),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(0.65),

    //Align to Algae
    CmdDriveToPoint(1.2_m, 0.15_m, 0_deg, 1.7_mps, true, 3_s),
    CmdPivotToPos(ALGAE_POS_INTAKE),
    // CmdWait(0.65),
    frc2::ParallelCommandGroup(
    CmdAlgaeIntake(-1.0),
    CmdDriveToPoint(1.5_m, 0.15_m, 0_deg, 1_mps, true, 2_s)),
    // CmdWait(1.0),
    // CmdElevatorToPosition(7),
    CmdDriveToPoint(1.2_m, 0.15_m, 0_deg, 1_mps, true, 3_s),

    //Go to Barge
    frc2::ParallelCommandGroup( CmdDriveToPoint(0.6_m, -1.0_m, -90_deg, 1.7_mps, false, 3_s),
    CmdElevatorToPosition(4) ),
    CmdDriveToPoint(0.0_m, -2.0_m, -180_deg, 1.7_mps, false, 3_s),
    CmdElevatorToPosition(9),
    CmdWait(0.7),
    CmdDriveToPoint(-0.1_m, -2.0_m, -180_deg, 1_mps, true, 3_s),
    CmdWait(0.15),
    CmdPivotIntake(1.0),

    //Get off line
    CmdElevatorToPosition(ELEV_POS_HOME),
    CmdPivotToPos(ALGAE_POS_CLOSE_HOME),
    CmdDriveToPoint(1.5_m, -2.0_m, -180_deg, 1.7_mps, true, 5_s),

    // //align to algae
    // // frc2::ParallelCommandGroup(
    //   CmdElevatorToPosition(1), 
    //   // CmdPivotToPos(6)),
    // CmdDriveToPoint(1.7_m, -1.5_m, 60_deg, 1.7_mps, false, 3_s),
    // frc2::ParallelCommandGroup(
    //   CmdDriveToPoint(2.1_m, -1.2_m, 60_deg, 1.7_mps, true, 3_s),
    //   CmdElevatorToPosition(8)),
    // CmdPivotToPos(3),
    // frc2::ParallelCommandGroup(
    //   CmdAlgaeIntake(-1.0),
    //   CmdDriveToPoint(2.4_m, -0.7_m, 60_deg, 1_mps, true, 3_s)),

    // //score in barge
    // CmdDriveToPoint(2.2_m, -0.9_m, 60_deg, 1_mps, true, 3_s),
    // CmdDriveToPoint(0.0_m, -2.0_m, 180_deg, 1.7_mps, true, 3_s),
    // CmdElevatorToPosition(9),
    // CmdWait(0.7),
    // CmdPivotIntake(1.0),
    // frc2::ParallelCommandGroup(
    //   CmdElevatorToPosition(1),
    //   CmdPivotToPos(6),
    //   CmdDriveToPoint(0.5_m, -2.0_m, 180_deg, 1.7_mps, true, 3_s)),

    CmdDriveClearAll(),
    CmdPrintText("Auto 1 Middle Alg End")
  );
}
