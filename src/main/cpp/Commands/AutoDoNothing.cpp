#include "Commands/AutoDoNothing.h"
#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "frc2/command/ParallelCommandGroup.h"

AutoDoNothing::AutoDoNothing() 
{
  // Add your commands here, e.g.
  AddCommands
  (
    CmdPrintText("AutoDoNothing"),
    CmdDriveToPoint(1.5_m, -1.5_m, 0_deg, 3.5_mps, false, 5_s),
    CmdElevatorToPosition(4),
    CmdDriveToPoint(2.4_m, -2.3_m, -60_deg, 2_mps, false, 5_s),
    CmdElevatorToPosition(3),
    CmdDriveToPoint(2.45_m, -2.48_m, -60_deg, 1_mps, true, 3_s),
    CmdWait(),
    CmdClawOuttake(-1.0),
    CmdElevatorToPosition(1),
    CmdWait(),
    CmdDriveToPoint(3.7_m, -1.4_m, -60_deg, 3.5_mps, false, 5_s),
    CmdDriveToPoint(6_m, -0.8_m, -125_deg, 3_mps, false, 5_s),
    CmdClawActivate(-1.0),
    CmdDriveToPoint(6.4_m, -0.2_m, -125_deg, 1.25_mps, true, 2_s),

    CmdPrintText("AutoDoNothing Done")
  );
}
