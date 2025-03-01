#include "Commands/AutoDoNothing.h"
#include "Commands/CmdPrintText.h"

AutoDoNothing::AutoDoNothing() 
{
  // Add your commands here, e.g.
  AddCommands
  (
    //Auto Setup 
    CmdPrintText("****** AutoDoNothing ******"),

    //CmdDriveClearAll(),
    //CmdGyroSetAngleOffset( 180.0),

    CmdPrintText("Did Nothing Successfully")
  );
}
