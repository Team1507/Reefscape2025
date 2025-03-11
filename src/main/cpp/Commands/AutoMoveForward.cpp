#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "Commands/CmdClawAuto.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "Commands/CmdDriveClearAll.h"
#include "Commands/AutoMoveForward.h"


AutoMoveForward::AutoMoveForward() 
{
  AddCommands(
    CmdPrintText("Auto 1 Right"),
    CmdDriveClearAll(),

    //Move Forward
    CmdDriveToPoint(0.5_m, 0_m, 0_deg, 1.5_mps, true, 5_s),
    
    CmdPrintText("Auto 1 Right Done")
  );
}
