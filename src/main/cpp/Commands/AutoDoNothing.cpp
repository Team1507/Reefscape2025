#include "Commands/AutoDoNothing.h"
#include "Commands/CmdPrintText.h"
#include "Commands/CmdDriveToPoint.h"

AutoDoNothing::AutoDoNothing() 
{
  // Add your commands here, e.g.
  AddCommands
  (
    CmdDriveToPoint(0_m, 0_m, 0_deg, 0_mps, true, 0_s)
  );
}
