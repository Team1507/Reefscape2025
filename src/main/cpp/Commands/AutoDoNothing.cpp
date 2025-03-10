#include "Commands/AutoDoNothing.h"
#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"
#include "Commands/CmdElevatorToPosition.h"
#include "Commands/CmdClawOuttake.h"
#include "Commands/CmdWait.h"
#include "Commands/CmdClawActivate.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "Commands/CmdDriveClearAll.h"

AutoDoNothing::AutoDoNothing() 
{
  // Add your commands here, e.g.
  AddCommands
  (
    CmdPrintText("AutoDoNothing"),
    CmdDriveClearAll(),

    //Nothing
    
    CmdPrintText("AutoDoNothing Done")
  );
}
