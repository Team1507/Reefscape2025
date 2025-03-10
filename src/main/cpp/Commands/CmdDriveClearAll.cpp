#include "Commands/CmdDriveClearAll.h"
#include "Robot.h"


CmdDriveClearAll::CmdDriveClearAll() 
{
  AddRequirements({&robotcontainer.driveSub});
}


void CmdDriveClearAll::Initialize() 
{
  robotcontainer.driveSub.ResetAllSensors();
}
